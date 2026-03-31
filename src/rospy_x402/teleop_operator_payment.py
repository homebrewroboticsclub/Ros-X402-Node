"""
Post-pay teleoperator in SOL using the same X402Client wallet as x402_buy_service.
"""

from __future__ import annotations

import json
import logging
from typing import Any, Dict, Tuple

logger = logging.getLogger(__name__)

SKIP_OPERATOR_PUBKEYS = frozenset(
    ("", "unknown", "pending_from_raid", "pending-from-raid")
)


def pay_operator_from_receipt_payload(
    x402_client: Any,
    receipt_payload: str,
    sol_per_sec: float,
) -> Tuple[bool, str, str]:
    """
    Parse KYR SignedReceipt JSON, compute duration * rate, send SOL to operator_pubkey.

    Returns (success, message, payment_signature).
    """
    try:
        receipt = json.loads(receipt_payload or "{}")
    except json.JSONDecodeError as e:
        return False, f"Invalid receipt JSON: {e}", ""

    if not isinstance(receipt, dict):
        return False, "Receipt payload must be a JSON object", ""

    operator_pubkey = str(receipt.get("operator_pubkey", "") or "").strip()
    if operator_pubkey.lower() in {x.lower() for x in SKIP_OPERATOR_PUBKEYS}:
        return True, "No valid operator Solana pubkey in receipt; payment skipped", ""

    started = int(receipt.get("started_at_sec", 0) or 0)
    ended = int(receipt.get("ended_at_sec", 0) or 0)
    duration_sec = max(ended - started, 0)
    amount_sol = float(duration_sec) * float(sol_per_sec)

    if amount_sol <= 0:
        return True, "Zero payment amount (duration or rate); skipped", "skipped_zero_amount"

    try:
        sig = x402_client.send_payment(operator_pubkey, amount_sol)
    except Exception as e:  # pylint: disable=broad-except
        logger.exception("Outgoing teleop operator payment failed")
        return False, str(e), ""

    logger.info(
        "Teleop operator payment: %s SOL for %s sec -> %s (sig=%s)",
        amount_sol,
        duration_sec,
        operator_pubkey,
        sig,
    )
    return True, "Payment sent", sig
