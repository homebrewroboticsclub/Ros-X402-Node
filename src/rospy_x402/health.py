import json
import logging
import os
import random
from typing import Dict, List, Optional

import rospkg

from .config import EndpointConfig, EndpointPricing, ServerConfig, load_config

# NOTE: Replace the random location mocks by subscribing to a GPS topic and updating the
# latest coordinates in this module to provide live robot positions.

_LOGGER = logging.getLogger(__name__)


def _mock_location() -> Dict[str, float]:
    return {
        "lat": round(random.uniform(22.28, 22.32), 6),
        "lng": round(random.uniform(114.15, 114.20), 6),
    }


def _config_path() -> Optional[str]:
    override_path = os.environ.get("ROSPY_X402_CONFIG")
    if override_path:
        return override_path

    try:
        package_path = rospkg.RosPack().get_path("rospy_x402")
    except rospkg.ResourceNotFound:
        _LOGGER.warning("rospy_x402 package path not found for health metadata lookup")
        return None

    return os.path.join(package_path, "config", "endpoints.example.json")


def _load_server_config() -> Optional[ServerConfig]:
    config_path = _config_path()
    if not config_path:
        return None

    try:
        return load_config(config_path)
    except FileNotFoundError:
        _LOGGER.warning("x402 health metadata config not found at %s", config_path)
    except json.JSONDecodeError as exc:
        _LOGGER.error("Failed to decode x402 config at %s: %s", config_path, exc)
    except Exception as exc:  # pylint: disable=broad-except
        _LOGGER.error("Unexpected error loading x402 config: %s", exc)

    return None


def _pricing_snapshot(pricing: Optional[EndpointPricing]) -> Optional[Dict[str, object]]:
    if not pricing:
        return None

    return {
        "amount": pricing.amount,
        "assetSymbol": pricing.asset_symbol,
        "receiverAccount": pricing.receiver_account,
        "paymentWindowSec": pricing.payment_window_sec,
    }


def _method_entry(endpoint: EndpointConfig) -> Dict[str, object]:
    params: Dict[str, object] = {}
    if endpoint.ros_action.args:
        params["args"] = endpoint.ros_action.args
    if endpoint.ros_action.kwargs:
        params["kwargs"] = endpoint.ros_action.kwargs

    return {
        "path": endpoint.path,
        "httpMethod": endpoint.http_method,
        "description": endpoint.description,
        "rosAction": {
            "module": endpoint.ros_action.module,
            "callable": endpoint.ros_action.callable,
        },
        "parameters": params,
        "pricing": _pricing_snapshot(endpoint.x402_pricing),
        "metadata": endpoint.metadata or None,
    }


def _available_methods() -> List[Dict[str, object]]:
    server_config = _load_server_config()
    if not server_config:
        return []

    return [_method_entry(endpoint) for endpoint in server_config.endpoints]


def get_health_status() -> Dict[str, object]:
    return {
        "status": "ready",
        "message": "Ready for commands",
        "availableMethods": _available_methods(),
        "location": _mock_location(),
    }

