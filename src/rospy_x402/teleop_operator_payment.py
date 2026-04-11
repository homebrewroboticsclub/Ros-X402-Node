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


def is_abnormal_closure_reason(closure_reason: str) -> bool:
    """Interim policy: fractional pay for watchdog / operator disconnect class."""
    c = (closure_reason or "").strip().lower()
    if not c:
        return False
    return c.startswith("robot_watchdog") or c.startswith("operator_disconnect")


def pay_operator_from_receipt_payload(
    x402_client: Any,
    receipt_payload: str,
    sol_per_sec: float,
    flat_sol: float = 0.0,
    abnormal_payment_fraction: float = 1.0,
) -> Tuple[bool, str, str, float]:
    """
    Parse KYR SignedReceipt JSON, compute amount, send SOL to operator_pubkey.

    Amount precedence:
    1) receipt.operator_payment_sol (> 0), e.g. from RAID SessionGrant
    2) else flat_sol rosparam if > 0 (fixed per session, ignores duration)
    3) else (ended_at_sec - started_at_sec) * sol_per_sec

    If receipt.closure_reason is abnormal (watchdog / operator_disconnect*) and
    abnormal_payment_fraction is in (0, 1), the computed amount is multiplied by that fraction.

    Returns (success, message, payment_signature, amount_sol_attempted).
    """
    try:
        receipt = json.loads(receipt_payload or "{}")
    except json.JSONDecodeError as e:
        return False, f"Invalid receipt JSON: {e}", "", 0.0

    if not isinstance(receipt, dict):
        return False, "Receipt payload must be a JSON object", "", 0.0

    operator_pubkey = str(receipt.get("operator_pubkey", "") or "").strip()
    if operator_pubkey.lower() in {x.lower() for x in SKIP_OPERATOR_PUBKEYS}:
        logger.warning(
            "Operator payment skipped: receipt has placeholder operator_pubkey=%r — "
            "RAID must return a signed SessionGrant with a real Solana base58 address.",
            operator_pubkey or "(empty)",
        )
        return True, "No valid operator Solana pubkey in receipt; payment skipped", "", 0.0

    started = int(receipt.get("started_at_sec", 0) or 0)
    ended = int(receipt.get("ended_at_sec", 0) or 0)
    duration_sec = max(ended - started, 0)

    raw_receipt_pay = receipt.get("operator_payment_sol")
    amount_sol = 0.0
    if isinstance(raw_receipt_pay, (int, float)) and float(raw_receipt_pay) > 0:
        amount_sol = float(raw_receipt_pay)
    elif flat_sol and float(flat_sol) > 0:
        amount_sol = float(flat_sol)
    else:
        amount_sol = float(duration_sec) * float(sol_per_sec)

    closure = str(receipt.get("closure_reason", "") or "")
    frac = float(abnormal_payment_fraction)
    if is_abnormal_closure_reason(closure) and 0.0 < frac < 1.0:
        before = amount_sol
        amount_sol = before * frac
        logger.info(
            "Teleop operator payment: abnormal closure_reason=%r — applying fraction %.2f (%s -> %s SOL)",
            closure,
            frac,
            before,
            amount_sol,
        )

    if amount_sol <= 0:
        return True, "Zero payment amount; skipped", "skipped_zero_amount", 0.0

    try:
        sig = x402_client.send_payment(operator_pubkey, amount_sol)
    except Exception as e:  # pylint: disable=broad-except
        logger.exception("Outgoing teleop operator payment failed")
        return False, str(e), "", amount_sol

    logger.info(
        "Teleop operator payment: %s SOL (duration=%s sec) -> %s (sig=%s)",
        amount_sol,
        duration_sec,
        operator_pubkey,
        sig,
    )
    return True, "Payment sent", sig, amount_sol
