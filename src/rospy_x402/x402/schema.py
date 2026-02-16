"""
x402 protocol V2 schema helpers (x402scan / Bazaar compatible).

See: https://x402scan.com/ register resource validation schema (V2).
"""

from typing import Any, Dict, List, Optional

# CAIP-2 network identifiers for Solana (genesis hash truncated to 32 chars)
SOLANA_MAINNET_NETWORK = "solana:5eykt4UsFv8P8NJdTREpY1vzqKqZKvdp"
SOLANA_DEVNET_NETWORK = "solana:EtWTRABZaYq6iMfeYKouRu166VU2xqa1"
SOLANA_TESTNET_NETWORK = "solana:4uhcVJyU9pJkvQyS88uRDiswHXSCkY3z"


def build_accepts_entry(
    *,
    network: str,
    amount: str,
    pay_to: str,
    max_timeout_seconds: int,
    asset: str,
    extra: Optional[Dict[str, Any]] = None,
    scheme: str = "exact",
) -> Dict[str, Any]:
    """
    Build a single entry for the x402 V2 `accepts` array.

    V2 requires `extra` to be present (use {} if no extra data).
    """
    return {
        "scheme": scheme,
        "network": network,
        "amount": amount,
        "payTo": pay_to,
        "maxTimeoutSeconds": max_timeout_seconds,
        "asset": asset,
        "extra": extra if extra is not None else {},
    }


def build_402_response_v2(
    *,
    accepts: List[Dict[str, Any]],
    error: Optional[str] = None,
    resource: Optional[Dict[str, Any]] = None,
    extensions: Optional[Dict[str, Any]] = None,
) -> Dict[str, Any]:
    """
    Build x402 V2 response body for HTTP 402 Payment Required.

    Compatible with x402scan validation schema.
    """
    payload: Dict[str, Any] = {
        "x402Version": 2,
        "accepts": accepts,
    }
    if error is not None:
        payload["error"] = error
    if resource is not None:
        payload["resource"] = resource
    if extensions is not None:
        payload["extensions"] = extensions
    return payload


def build_resource_entry(
    url: str,
    description: str = "",
    mime_type: str = "application/json",
) -> Dict[str, Any]:
    """Build x402 V2 `resource` object."""
    return {
        "url": url,
        "description": description,
        "mimeType": mime_type,
    }


def build_bazaar_extension(
    *,
    input_schema: Optional[Any] = None,
    output_schema: Optional[Any] = None,
    example_input: Optional[Any] = None,
    example_output: Optional[Any] = None,
) -> Dict[str, Any]:
    """
    Build extensions.bazaar for discoverable APIs (x402scan UI).

    See Validation Schema: bazaar.info.input, bazaar.info.output, bazaar.schema.
    """
    bazaar: Dict[str, Any] = {}
    if example_input is not None or example_output is not None:
        bazaar["info"] = {}
        if example_input is not None:
            bazaar["info"]["input"] = example_input
        if example_output is not None:
            bazaar["info"]["output"] = example_output
    if input_schema is not None or output_schema is not None:
        bazaar["schema"] = {}
        if input_schema is not None:
            bazaar["schema"]["input"] = input_schema
        if output_schema is not None:
            bazaar["schema"]["output"] = output_schema
    return {"bazaar": bazaar} if bazaar else {}
