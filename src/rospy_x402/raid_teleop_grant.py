"""
Parse RAID teleop/help JSON for a KYR SessionGrant + Ed25519 signature.

When RAID implements the full cycle, it returns canonical grant bytes and a base58
signature. Otherwise the escalation layer falls back to a local mock grant.
"""

from __future__ import annotations

import json
import logging
from typing import Any, Dict, Optional, Tuple

logger = logging.getLogger(__name__)

# Preferred: RAID signs exact UTF-8 bytes of this string (no re-serialization on robot).
_PAYLOAD_KEYS = (
    "teleopGrantPayload",
    "grantPayload",
    "sessionGrantPayload",
)
_SIG_KEYS = (
    "teleopGrantSignature",
    "grantSignature",
    "sessionGrantSignature",
)


def _first_str(d: Dict[str, Any], keys: Tuple[str, ...]) -> str:
    for k in keys:
        v = d.get(k)
        if isinstance(v, str) and v.strip():
            return v.strip()
    return ""


def _merge_nested_top_level(data: Dict[str, Any]) -> Dict[str, Any]:
    """Flatten common nested shapes so keys are visible at top level."""
    out = dict(data)
    for nest_key in ("helpRequest", "help_request", "teleopHelp", "data"):
        nested = data.get(nest_key)
        if isinstance(nested, dict):
            for k, v in nested.items():
                if k not in out or out[k] in (None, "", {}):
                    out[k] = v
    return out


def canonical_session_grant_json(grant_obj: Dict[str, Any]) -> str:
    """
    Canonical JSON for signing (RAID must sign these exact bytes if using object form).
    """
    return json.dumps(grant_obj, sort_keys=True, separators=(",", ":"), ensure_ascii=False)


def extract_signed_grant_from_raid_help_response(data: Any) -> Optional[Tuple[str, str]]:
    """
    Returns (grant_payload_str, signature_b58) if RAID supplied a grant + signature.

    If None, EscalationManager should build the legacy mock grant.
    """
    if not isinstance(data, dict):
        return None

    flat = _merge_nested_top_level(data)
    payload = _first_str(flat, _PAYLOAD_KEYS)
    signature = _first_str(flat, _SIG_KEYS)

    if payload and signature:
        return payload, signature

    # Structured grant object + signature (RAID signs canonical_session_grant_json(grant)).
    grant_obj = flat.get("sessionGrant")
    if not isinstance(grant_obj, dict):
        grant_obj = flat.get("session_grant")
    if isinstance(grant_obj, dict) and signature:
        return canonical_session_grant_json(grant_obj), signature

    logger.debug(
        "RAID teleop/help: no signed grant fields (expected teleopGrantPayload+teleopGrantSignature or sessionGrant+teleopGrantSignature)"
    )
    return None
