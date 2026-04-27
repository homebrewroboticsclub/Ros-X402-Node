"""
RAID App HTTP integration: enroll, on-disk credentials, operator allowlist file.

Kept free of rospy for unit testing. See DOC/RAID_INTEGRATION.md.
"""

from __future__ import annotations

import json
import os
import time
from typing import Any, Dict, List, Optional, Tuple

import requests

DEFAULT_STATE_BASENAME = "raid_robot_state.json"
DEFAULT_ALLOWLIST_BASENAME = "raid_operator_allowlist.json"


def default_state_path() -> str:
    base = os.path.join(os.path.expanduser("~"), ".ros")
    os.makedirs(base, mode=0o755, exist_ok=True)
    return os.path.join(base, DEFAULT_STATE_BASENAME)


def default_allowlist_path(state_path: str) -> str:
    d = os.path.dirname(os.path.abspath(state_path))
    return os.path.join(d, DEFAULT_ALLOWLIST_BASENAME)


def normalized_sync_path(path: str) -> str:
    p = (path or "").strip()
    if not p.startswith("/"):
        p = "/" + p
    if len(p) > 1 and p.endswith("/"):
        p = p.rstrip("/")
    return p


def build_operator_registry_url(host: str, port: int, sync_path: str) -> str:
    h = (host or "").strip()
    path = normalized_sync_path(sync_path)
    return f"http://{h}:{int(port)}{path}"


def load_raid_state(path: str) -> Optional[Dict[str, Any]]:
    if not path or not os.path.isfile(path):
        return None
    try:
        with open(path, "r", encoding="utf-8") as f:
            data = json.load(f)
    except (OSError, json.JSONDecodeError):
        return None
    if not isinstance(data, dict):
        return None
    return data


def save_raid_state(path: str, robot_id: str, teleop_secret: str) -> None:
    d = os.path.dirname(os.path.abspath(path))
    if d:
        os.makedirs(d, mode=0o755, exist_ok=True)
    payload = {
        "id": robot_id,
        "teleopSecret": teleop_secret,
        "saved_at": int(time.time()),
    }
    _atomic_write_json(path, payload)


def credentials_from_state(data: Dict[str, Any]) -> Optional[Tuple[str, str]]:
    rid = (data.get("id") or data.get("robotId") or "").strip()
    sec = (data.get("teleopSecret") or data.get("teleop_secret") or "").strip()
    if rid and sec:
        return rid, sec
    return None


def build_enroll_body(
    enrollment_key: str,
    host: str,
    port: int,
    *,
    rosbridge_host: Optional[str] = None,
    rosbridge_port: Optional[int] = None,
    name: Optional[str] = None,
    teleop_secret: Optional[str] = None,
    operator_registry_url: Optional[str] = None,
) -> Dict[str, Any]:
    body: Dict[str, Any] = {
        "enrollmentKey": enrollment_key,
        "host": host,
        "port": int(port),
    }
    if rosbridge_host is not None:
        body["rosbridgeHost"] = rosbridge_host
    if rosbridge_port is not None:
        body["rosbridgePort"] = int(rosbridge_port)
    if name:
        body["name"] = name
    if teleop_secret:
        body["teleopSecret"] = teleop_secret
    if operator_registry_url:
        body["operatorRegistryUrl"] = operator_registry_url
    return body


def post_robot_enroll(
    raid_base_url: str,
    fleet_secret: str,
    body: Dict[str, Any],
    timeout: float = 30.0,
) -> Dict[str, Any]:
    """
    POST /api/robots/enroll with fleet auth.
    Raises requests.HTTPError on HTTP error.
    """
    base = raid_base_url.rstrip("/")
    url = f"{base}/api/robots/enroll"
    headers = {
        "Content-Type": "application/json",
        "X-Robot-Fleet-Secret": fleet_secret,
    }
    resp = requests.post(url, json=body, headers=headers, timeout=timeout)
    resp.raise_for_status()
    data = resp.json()
    if not isinstance(data, dict):
        raise ValueError("RAID enroll response is not a JSON object")
    return data


def enroll_robot(
    raid_base_url: str,
    fleet_secret: str,
    enrollment_key: str,
    advertise_host: str,
    advertise_port: int,
    *,
    rosbridge_host: Optional[str] = None,
    rosbridge_port: Optional[int] = None,
    name: Optional[str] = None,
    teleop_secret: Optional[str] = None,
    operator_registry_url: Optional[str] = None,
    timeout: float = 30.0,
) -> Dict[str, Any]:
    body = build_enroll_body(
        enrollment_key,
        advertise_host,
        advertise_port,
        rosbridge_host=rosbridge_host,
        rosbridge_port=rosbridge_port,
        name=name,
        teleop_secret=teleop_secret,
        operator_registry_url=operator_registry_url,
    )
    return post_robot_enroll(raid_base_url, fleet_secret, body, timeout=timeout)


def parse_enroll_credentials(data: Dict[str, Any]) -> Tuple[str, str]:
    rid = (data.get("id") or "").strip()
    sec = (data.get("teleopSecret") or "").strip()
    if not rid or not sec:
        raise ValueError("RAID enroll response missing id or teleopSecret")
    return rid, sec


def save_operator_allowlist(path: str, teleoperator_ids: List[str]) -> None:
    d = os.path.dirname(os.path.abspath(path))
    if d:
        os.makedirs(d, mode=0o755, exist_ok=True)
    payload = {
        "allowedTeleoperatorIds": list(teleoperator_ids),
        "updated_at": int(time.time()),
    }
    _atomic_write_json(path, payload)


def load_operator_allowlist(path: str) -> List[str]:
    if not path or not os.path.isfile(path):
        return []
    try:
        with open(path, "r", encoding="utf-8") as f:
            data = json.load(f)
    except (OSError, json.JSONDecodeError):
        return []
    if not isinstance(data, dict):
        return []
    raw = data.get("allowedTeleoperatorIds")
    if not isinstance(raw, list):
        return []
    return [str(x).strip() for x in raw if str(x).strip()]


def _atomic_write_json(path: str, payload: Dict[str, Any]) -> None:
    d = os.path.dirname(os.path.abspath(path))
    if d:
        os.makedirs(d, mode=0o755, exist_ok=True)
    tmp = f"{path}.tmp.{os.getpid()}"
    try:
        with open(tmp, "w", encoding="utf-8") as f:
            json.dump(payload, f, indent=2, sort_keys=True)
            f.write("\n")
        os.replace(tmp, path)
    finally:
        if os.path.isfile(tmp):
            try:
                os.remove(tmp)
            except OSError:
                pass


def parse_raid_robot_push_body(
    body: Dict[str, Any],
) -> Tuple[Optional[List[str]], Optional[Dict[str, Any]]]:
    """
    RAID -> robot POST body (operator allowlist + optional DATA_NODE batch config).
    At least one of allowedTeleoperatorIds or dataNodeSync must be present.
    """
    ids_opt: Optional[List[str]] = None
    if "allowedTeleoperatorIds" in body:
        raw = body["allowedTeleoperatorIds"]
        if not isinstance(raw, list):
            raise ValueError("allowedTeleoperatorIds must be a list")
        ids_opt = [str(x).strip() for x in raw if str(x).strip()]
    dn_opt: Optional[Dict[str, Any]] = None
    if "dataNodeSync" in body:
        dns = body["dataNodeSync"]
        if dns is None:
            dn_opt = None
        elif isinstance(dns, dict):
            dn_opt = dns
        else:
            raise ValueError("dataNodeSync must be a JSON object")
    if ids_opt is None and dn_opt is None:
        raise ValueError("Expected allowedTeleoperatorIds and/or dataNodeSync")
    return ids_opt, dn_opt


def parse_operator_sync_body(body: Dict[str, Any]) -> List[str]:
    raw = body.get("allowedTeleoperatorIds")
    if raw is None:
        raise ValueError("Missing allowedTeleoperatorIds")
    if not isinstance(raw, list):
        raise ValueError("allowedTeleoperatorIds must be a list")
    return [str(x).strip() for x in raw if str(x).strip()]
