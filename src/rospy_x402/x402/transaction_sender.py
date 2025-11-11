import logging
from typing import Any, Dict

from .exceptions import PaymentSubmissionError

logger = logging.getLogger(__name__)

try:
    from solana.rpc.api import Client as SolanaClient
    from solana.rpc.types import TxOpts
    from solders.hash import Hash
    from solders.keypair import Keypair
    from solders.message import Message
    from solders.pubkey import Pubkey
    from solders.system_program import TransferParams, transfer
    from solders.transaction import Transaction as SoldersTransaction
except ImportError as exc:  # pragma: no cover - optional dependency
    logger.warning(
        "solana package is not available. Outgoing payments will fail until it is installed."
    )
    SolanaClient = None  # type: ignore
    Keypair = None  # type: ignore
    Pubkey = None  # type: ignore
    TransferParams = None  # type: ignore
    Message = None  # type: ignore
    Hash = None  # type: ignore
    SoldersTransaction = None  # type: ignore
    _IMPORT_ERROR = exc
else:
    _IMPORT_ERROR = None


class SolanaTransactionSender:
    def __init__(self, rpc_endpoint: str) -> None:
        if any(
            dependency is None
            for dependency in (
                SolanaClient,
                Keypair,
                Pubkey,
                TransferParams,
                Message,
                Hash,
                SoldersTransaction,
            )
        ):
            raise PaymentSubmissionError(
                "solana Python package is required for outgoing payments."
            ) from _IMPORT_ERROR
        self._client = SolanaClient(rpc_endpoint)

    def transfer_lamports(
        self,
        keypair_secret: bytes,
        source_public_key: str,
        destination_public_key: str,
        lamports: int,
        commitment: str = "confirmed",
    ) -> Dict[str, Any]:
        if lamports <= 0:
            raise PaymentSubmissionError("Transfer amount must be positive.")

        try:
            keypair = Keypair.from_bytes(keypair_secret)
        except Exception as exc:  # pragma: no cover - library exception
            raise PaymentSubmissionError("Failed to load keypair for payment submission.") from exc

        try:
            source_pubkey = Pubkey.from_string(source_public_key)
            destination_pubkey = Pubkey.from_string(destination_public_key)
        except Exception as exc:  # pragma: no cover
            raise PaymentSubmissionError("Invalid public key for transfer.") from exc

        instruction = transfer(
            TransferParams(
                from_pubkey=source_pubkey,
                to_pubkey=destination_pubkey,
                lamports=lamports,
            )
        )

        message = Message([instruction], source_pubkey)

        try:
            blockhash_resp = self._client.get_latest_blockhash(commitment=commitment)
            recent_blockhash = blockhash_resp.value.blockhash
        except Exception as exc:  # pragma: no cover
            raise PaymentSubmissionError("Failed to fetch recent blockhash.") from exc

        if not isinstance(recent_blockhash, Hash):
            try:
                recent_blockhash = Hash.from_string(str(recent_blockhash))
            except Exception as exc:  # pragma: no cover
                raise PaymentSubmissionError("Invalid blockhash value from RPC.") from exc

        transaction = SoldersTransaction([keypair], message, recent_blockhash)

        try:
            response = self._client.send_transaction(
                transaction,
                opts=TxOpts(
                    skip_confirmation=False,
                    preflight_commitment=commitment,
                ),
            )
        except Exception as exc:  # pragma: no cover
            raise PaymentSubmissionError("Failed to submit transaction to Solana RPC.") from exc

        logger.info(
            "Submitted transfer transaction signature=%s destination=%s lamports=%s",
            response.value,
            destination_public_key,
            lamports,
        )
        return {"result": response.value}

