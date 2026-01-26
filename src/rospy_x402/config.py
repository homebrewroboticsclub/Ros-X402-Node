import json
import os
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional


@dataclass
class EndpointAction:
    type: str
    module: str
    callable: str
    args: List[Any] = field(default_factory=list)
    kwargs: Dict[str, Any] = field(default_factory=dict)


@dataclass
class EndpointPricing:
    amount: float
    asset_symbol: str
    receiver_account: str
    payment_window_sec: int = 300


@dataclass
class EndpointConfig:
    path: str
    http_method: str
    description: Optional[str]
    ros_action: EndpointAction
    x402_pricing: Optional[EndpointPricing]
    metadata: Dict[str, Any] = field(default_factory=dict)


@dataclass
class ServerConfig:
    listen_host: str = "0.0.0.0"
    listen_port: int = 18080
    require_https: bool = False
    facilitator_url: Optional[str] = None
    endpoints: List[EndpointConfig] = field(default_factory=list)


def _load_json(path: str) -> Dict[str, Any]:
    with open(path, "r", encoding="utf-8") as config_fh:
        return json.load(config_fh)


def load_config(path: str) -> ServerConfig:
    if not os.path.isfile(path):
        raise FileNotFoundError(f"Configuration file does not exist: {path}")

    raw = _load_json(path)
    endpoints: List[EndpointConfig] = []
    for endpoint in raw.get("endpoints", []):
        action_raw = endpoint.get("ros_action", {})
        pricing_raw = endpoint.get("x402_pricing")

        action = EndpointAction(
            type=action_raw.get("type", "python_call"),
            module=action_raw.get("module", ""),
            callable=action_raw.get("callable", ""),
            args=action_raw.get("args", []) or [],
            kwargs=action_raw.get("kwargs", {}) or {},
        )

        pricing = None
        if pricing_raw:
            pricing = EndpointPricing(
                amount=float(pricing_raw.get("amount", 0.0)),
                asset_symbol=pricing_raw.get("asset_symbol", "SOL"),
                receiver_account=pricing_raw.get("receiver_account", ""),
                payment_window_sec=int(pricing_raw.get("payment_window_sec", 300)),
            )

        endpoints.append(
            EndpointConfig(
                path=endpoint.get("path", ""),
                http_method=(endpoint.get("http_method") or "GET").upper(),
                description=endpoint.get("description"),
                ros_action=action,
                x402_pricing=pricing,
                metadata=endpoint.get("metadata", {}) or {},
            )
        )

    return ServerConfig(
        listen_host=raw.get("listen_host", "0.0.0.0"),
        listen_port=int(raw.get("listen_port", 18080)),
        require_https=bool(raw.get("require_https", False)),
        facilitator_url=raw.get("facilitator_url"),
        endpoints=endpoints,
    )

