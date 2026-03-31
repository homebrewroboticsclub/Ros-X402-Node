"""
HTTP client for RAID peaq claim endpoints (see br-vr-dev-sinc/DOC/RAID_APP_PEAQ_CLAIM_SPEC.md).
"""

import logging
import time
from typing import Any, Dict, Optional

import requests

logger = logging.getLogger(__name__)


def fetch_peaq_claim(
    raid_app_url: str,
    robot_id: str,
    teleop_secret: str,
    help_request_id: str,
    timeout_sec: float = 10.0,
    poll_attempts: int = 3,
    poll_delay_sec: float = 1.0,
    session: Optional[requests.Session] = None,
) -> Optional[Dict[str, Any]]:
    """
    GET /api/robots/{robotId}/peaq/claim?helpRequestId=...
    Returns peaq_claim dict or None on failure / not ready (after polls).
    """
    if not help_request_id or not robot_id or not teleop_secret:
        return None
    base = raid_app_url.rstrip("/")
    url = f"{base}/api/robots/{robot_id}/peaq/claim"
    headers = {
        "X-Robot-Teleop-Secret": teleop_secret,
    }
    sess = session or requests.Session()
    params = {"helpRequestId": help_request_id}
    last_err = None
    for attempt in range(max(1, poll_attempts)):
        try:
            resp = sess.get(url, headers=headers, params=params, timeout=timeout_sec)
            if resp.status_code == 200:
                data = resp.json()
                claim = data.get("peaq_claim")
                if isinstance(claim, dict):
                    return claim
                logger.warning("RAID peaq/claim 200 but peaq_claim missing or not object")
                return None
            if resp.status_code == 404:
                logger.info(
                    "RAID peaq/claim not ready (404), attempt %s/%s",
                    attempt + 1,
                    poll_attempts,
                )
                last_err = "404"
                if attempt + 1 < poll_attempts:
                    time.sleep(max(0.1, poll_delay_sec))
                continue
            logger.warning("RAID peaq/claim HTTP %s: %s", resp.status_code, resp.text[:500])
            return None
        except requests.RequestException as e:
            last_err = str(e)
            logger.warning("RAID peaq/claim request error: %s", e)
            if attempt + 1 < poll_attempts:
                time.sleep(max(0.1, poll_delay_sec))
    if last_err:
        logger.info("RAID peaq/claim gave up after polls (%s)", last_err)
    return None
