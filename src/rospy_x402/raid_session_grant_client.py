"""
Poll RAID GET …/teleop/session-grant after POST …/teleop/help (operator must Accept first).

See DOC/ROBOT_TELEOP_KYR_RAID_GRANT.md (RAID x402_raid_app contract).
"""

from __future__ import annotations

import logging
import time
from typing import Any, Dict, Optional, Tuple
from urllib.parse import urljoin

import requests

from rospy_x402.raid_teleop_grant import (
    _merge_nested_top_level,
    extract_signed_grant_from_raid_help_response,
)

logger = logging.getLogger(__name__)

_SIGNER_KEYS = ("grantSignerPublicKey", "grant_signer_public_key", "teleopGrantSignerPublicKey")


def _first_signer_pubkey(data: Dict[str, Any]) -> Optional[str]:
    flat = _merge_nested_top_level(data)
    for k in _SIGNER_KEYS:
        v = flat.get(k)
        if isinstance(v, str) and v.strip():
            return v.strip()
    return None


def _resolve_session_grant_get_url(
    raid_app_url: str,
    robot_id: str,
    help_request_id: str,
    help_response: Dict[str, Any],
) -> str:
    base = raid_app_url.rstrip("/")
    flat = _merge_nested_top_level(help_response)
    raw = flat.get("teleopGrantPollUrl") or flat.get("teleop_grant_poll_url")
    if isinstance(raw, str) and raw.strip():
        path = raw.strip()
        path = (
            path.replace("{robotId}", robot_id)
            .replace("{helpRequestId}", help_request_id)
            .replace("{id}", help_request_id)
        )
        if path.startswith("http://") or path.startswith("https://"):
            return path
        return urljoin(base + "/", path.lstrip("/"))
    return f"{base}/api/robots/{robot_id}/teleop/session-grant?helpRequestId={help_request_id}"


def poll_raid_session_grant(
    raid_app_url: str,
    robot_id: str,
    teleop_secret: str,
    help_request_id: str,
    help_response: Dict[str, Any],
    timeout_sec: float,
    interval_sec: float,
    request_timeout_sec: float = 15.0,
) -> Tuple[str, str, Optional[str]]:
    """
    Poll GET session-grant until 200 with teleopGrantPayload + teleopGrantSignature.

    Returns (payload_str, signature_b58, grant_signer_public_key_or_none).

    Raises:
        ValueError on 401, unrecoverable 404 (grant_unconfigured, grant_absent), HTTP errors, or timeout.
    """
    url = _resolve_session_grant_get_url(raid_app_url, robot_id, help_request_id, help_response)
    headers = {
        "X-Robot-Teleop-Secret": teleop_secret,
    }
    deadline = time.monotonic() + max(1.0, timeout_sec)
    interval = max(0.2, interval_sec)
    attempt = 0

    logger.info(
        "RAID session-grant poll start helpRequestId=%s url=%s timeout=%ss",
        help_request_id,
        url,
        timeout_sec,
    )

    while time.monotonic() < deadline:
        attempt += 1
        try:
            resp = requests.get(url, headers=headers, timeout=request_timeout_sec)
        except requests.RequestException as e:
            logger.warning("session-grant GET error (attempt %s): %s", attempt, e)
            time.sleep(interval)
            continue

        if resp.status_code == 401:
            raise ValueError(
                "RAID session-grant rejected credentials (401). Check teleop secret."
            )

        if resp.status_code == 200:
            try:
                data = resp.json()
            except ValueError as e:
                raise ValueError(f"RAID session-grant 200 but invalid JSON: {e}") from e
            signed = extract_signed_grant_from_raid_help_response(data)
            if signed:
                signer = _first_signer_pubkey(data)
                if signer:
                    logger.info(
                        "RAID session-grant ready (attempt %s). grantSignerPublicKey=%s — "
                        "must match KYR ~trusted_raid_keys if signature verification is enabled.",
                        attempt,
                        signer,
                    )
                else:
                    logger.info("RAID session-grant ready (attempt %s) without grantSignerPublicKey field.", attempt)
                return signed[0], signed[1], signer
            logger.warning(
                "RAID session-grant 200 but no teleopGrantPayload+signature; keys=%s",
                sorted(data.keys()) if isinstance(data, dict) else type(data),
            )
            time.sleep(interval)
            continue

        if resp.status_code >= 500:
            logger.warning(
                "RAID session-grant HTTP %s (attempt %s), retry after backoff",
                resp.status_code,
                attempt,
            )
            time.sleep(interval)
            continue

        if resp.status_code == 404:
            err_code = None
            try:
                body = resp.json()
                if isinstance(body, dict):
                    err_code = body.get("error")
            except ValueError:
                pass
            if err_code == "grant_not_ready":
                logger.info(
                    "RAID session-grant grant_not_ready (404), attempt %s; waiting for operator Accept…",
                    attempt,
                )
                time.sleep(interval)
                continue
            if err_code in ("grant_unconfigured", "grant_absent"):
                raise ValueError(
                    f"RAID session-grant cannot issue grant ({err_code}). "
                    "Fix RAID TELEOP_GRANT_SIGNING_SECRET_KEY or operator wallet_public_key. "
                    "See DOC/ROBOT_TELEOP_KYR_RAID_GRANT.md."
                )
            raise ValueError(
                f"RAID session-grant 404 error={err_code!r} body={resp.text[:400]!r}"
            )

        raise ValueError(f"RAID session-grant HTTP {resp.status_code}: {resp.text[:500]!r}")

    raise ValueError(
        f"RAID session-grant not ready within {timeout_sec}s (operator must Accept in RAID UI). "
        "Increase ~raid_session_grant_timeout_sec if needed. See DOC/ROBOT_TELEOP_KYR_RAID_GRANT.md."
    )
