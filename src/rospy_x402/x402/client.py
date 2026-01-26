import logging
import time
from typing import Any, Dict, Optional, Tuple

from .exceptions import (
    PaymentSubmissionError,
    PaymentTimeoutError,
    PaymentVerificationError,
)
from .key_manager import KeyMaterial
from .models import PaymentRequest, PaymentSession
from .solana_rpc import SolanaRpcClient
from .transaction_sender import SolanaTransactionSender

logger = logging.getLogger(__name__)

LAMPORTS_PER_SOL = 1_000_000_000


class X402Client:
    def __init__(self, rpc_endpoint: str = "https://api.mainnet-beta.solana.com"):
        self._rpc = SolanaRpcClient(endpoint=rpc_endpoint)
        self._sessions: Dict[str, PaymentSession] = {}
        self._key_material: Optional[KeyMaterial] = None
        self._signature_cache: Dict[str, Tuple[float, Dict[str, Any]]] = {}
        self._cache_ttl_sec = 0.3
        self._signature_fetch_limit = 200

    @property
    def public_key(self) -> Optional[str]:
        if self._key_material:
            return self._key_material.public_key_b58
        return None

    def set_key_material(self, key_material: KeyMaterial) -> None:
        self._key_material = key_material

    def create_payment_session(
        self, request: PaymentRequest, payment_window_sec: int
    ) -> PaymentSession:
        session = PaymentSession.create(request, ttl_seconds=payment_window_sec)
        self._sessions[request.reference] = session
        logger.info(
            "Created x402 payment session reference=%s amount=%s asset=%s receiver=%s",
            request.reference,
            request.amount,
            request.asset_symbol,
            request.receiver_account,
        )
        return session

    def get_session(self, reference: str) -> Optional[PaymentSession]:
        return self._sessions.get(reference)

    def verify_payment(self, reference: str) -> PaymentSession:
        session = self._sessions.get(reference)
        if session is None:
            raise PaymentVerificationError(f"Unknown payment reference: {reference}")

        if session.is_expired:
            raise PaymentTimeoutError(
                f"Payment window expired for reference {reference}"
            )

        lamports_expected = int(session.request.amount * LAMPORTS_PER_SOL)
        receiver = session.request.receiver_account

        now = time.time()
        cache_entry = self._signature_cache.get(receiver)
        if cache_entry and now - cache_entry[0] < self._cache_ttl_sec:
            signatures = cache_entry[1]
        else:
            signatures = self._rpc.get_signatures_for_address(
                receiver, limit=self._signature_fetch_limit
            )
            self._signature_cache[receiver] = (now, signatures)

        if not signatures.get("result"):
            logger.debug(
                "No signatures found for receiver=%s reference=%s expected_lamports=%s",
                receiver,
                reference,
                lamports_expected,
            )

        for signature_info in signatures.get("result", []):
            signature = signature_info.get("signature")
            if not signature:
                continue
            logger.debug(
                "Inspecting signature=%s reference=%s receiver=%s",
                signature,
                reference,
                receiver,
            )

            tx = self._rpc.get_transaction(signature)
            result = tx.get("result")
            if not result:
                logger.debug(
                    "Transaction %s has no result yet for reference=%s", signature, reference
                )
                continue

            meta = result.get("meta") or {}
            status = meta.get("err")
            if status is not None:
                continue

            message = result.get("transaction", {}).get("message", {})
            instructions = message.get("instructions", [])
            for instruction in instructions:
                parsed = instruction.get("parsed")
                if not parsed:
                    continue
                info = parsed.get("info", {})
                if parsed.get("type") == "transfer":
                    destination = info.get("destination")
                    amount_raw = info.get("lamports")
                    if (
                        destination == session.request.receiver_account
                        and isinstance(amount_raw, (int, float, str))
                    ):
                        lamports_received = int(amount_raw)
                        if lamports_received >= lamports_expected:
                            session.signature = signature
                            logger.info(
                                "Verified x402 payment signature=%s reference=%s received=%s lamports expected=%s",
                                signature,
                                reference,
                                lamports_received,
                                lamports_expected,
                            )
                            return session

        raise PaymentVerificationError(
            f"Could not verify payment for reference {reference} (expected {lamports_expected} lamports to {receiver})"
        )

    def send_payment(self, destination: str, amount_sol: float) -> str:
        if self._key_material is None:
            raise PaymentSubmissionError(
                "Private key not available. Cannot send payment."
            )

        lamports = int(amount_sol * LAMPORTS_PER_SOL)
        sender = SolanaTransactionSender(self._rpc.endpoint)
        response = sender.transfer_lamports(
            keypair_secret=self._key_material.secret_key_64,
            source_public_key=self._key_material.public_key_b58,
            destination_public_key=destination,
            lamports=lamports,
        )
        signature = response.get("result")
        if not signature:
            raise PaymentSubmissionError("Solana RPC did not return a transaction signature.")
        logger.info(
            "Sent x402 payment signature=%s destination=%s amount=%s lamports",
            signature,
            destination,
            lamports,
        )
        return signature

