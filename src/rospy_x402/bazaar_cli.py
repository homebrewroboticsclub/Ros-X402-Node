"""
Console utility for x402 Bazaar: search and configure paid services for robot outgoing payments.

Use this to discover x402-enabled APIs and generate configuration or rosservice call templates
so the robot can pay for external functionality (e.g. via x402_buy_service).

Default discovery API: CDP Coinbase (configurable via --api-url).
"""

import argparse
import json
import sys
import urllib.error
import urllib.request
from typing import Any, Dict, List, Optional, Union


DEFAULT_BAZAAR_API = "https://api.cdp.coinbase.com/platform/v2/x402/discovery/resources"


def _http_get(url: str, timeout: int = 15) -> Dict[str, Any]:
    req = urllib.request.Request(url, method="GET")
    req.add_header("Accept", "application/json")
    with urllib.request.urlopen(req, timeout=timeout) as resp:
        return json.loads(resp.read().decode("utf-8"))


def _http_get_with_follow(
    url: str, method: str = "GET", body: Optional[str] = None, timeout: int = 15
) -> tuple:
    """Returns (status_code, response_body_dict_or_str)."""
    data = body.encode("utf-8") if body else None
    req = urllib.request.Request(url, data=data, method=method)
    req.add_header("Accept", "application/json")
    if body:
        req.add_header("Content-Type", "application/json")
    try:
        with urllib.request.urlopen(req, timeout=timeout) as resp:
            return resp.status, json.loads(resp.read().decode("utf-8"))
    except urllib.error.HTTPError as exc:
        body_b = exc.read()
        try:
            payload = json.loads(body_b.decode("utf-8"))
            return exc.code, payload
        except (ValueError, UnicodeDecodeError):
            return exc.code, body_b.decode("utf-8", errors="replace")


def cmd_search(api_url: str, limit: int, offset: int) -> None:
    """List resources from the x402 Bazaar discovery API."""
    params = []
    if limit > 0:
        params.append(f"limit={limit}")
    if offset > 0:
        params.append(f"offset={offset}")
    url = api_url + ("?" + "&".join(params) if params else "")
    try:
        data = _http_get(url)
    except (urllib.error.URLError, urllib.error.HTTPError, json.JSONDecodeError) as exc:
        print(f"Error fetching discovery: {exc}", file=sys.stderr)
        sys.exit(1)

    raw = data.get("resources")
    resources: List[Dict[str, Any]] = raw if isinstance(raw, list) else []
    total = data.get("total", len(resources))
    print(f"Resources (total={total}, limit={data.get('limit', limit)}, offset={data.get('offset', offset)}):\n")
    for r in resources:
        url_val = r.get("url", "")
        meta = r.get("metadata") or {}
        desc = (meta.get("description") or "")[:60]
        rtype = r.get("type", "http")
        print(f"  {url_val}")
        print(f"    type={rtype}  {desc}")
    if not resources:
        print("  (none)")


def cmd_show(resource_url: str, timeout: int) -> None:
    """Probe a resource URL and show x402 payment requirements (e.g. 402 response)."""
    status, payload = _http_get_with_follow(resource_url, method="GET", timeout=timeout)
    if status == 402 and isinstance(payload, dict):
        print("Payment required (402):")
        print(json.dumps(payload, indent=2))
        if payload.get("x402Version") == 2 and payload.get("accepts"):
            acc = payload["accepts"][0]
            print("\nTo pay and call from the robot, use x402_buy_service with:")
            print(f"  endpoint: '{resource_url}'")
            print(f"  amount: {acc.get('amount', '?')}")
            print(f"  payTo (receiver): {acc.get('payTo', '?')}")
            extra = acc.get("extra") or {}
            if extra.get("reference"):
                print(f"  (reference from accepts[0].extra: {extra.get('reference')})")
        return
    if status == 200 and isinstance(payload, dict):
        print("Resource responded 200 OK (no payment required for this request):")
        print(json.dumps(payload, indent=2)[:1024])
        return
    print(f"Status: {status}")
    print(payload if isinstance(payload, str) else json.dumps(payload, indent=2))


def cmd_configure(resource_url: str, method: str, output_path: Optional[str], timeout: int) -> None:
    """
    Fetch 402 from resource URL and output a configuration snippet or rosservice call template
    for the robot to pay this service.
    """
    status, payload = _http_get_with_follow(
        resource_url, method=method, body="{}" if method == "POST" else None, timeout=timeout
    )
    if status != 402 or not isinstance(payload, dict) or payload.get("x402Version") != 2:
        print("Resource did not return x402 V2 402 response.", file=sys.stderr)
        if status == 402:
            print("Response:", json.dumps(payload, indent=2), file=sys.stderr)
        else:
            print(f"Status: {status}", file=sys.stderr)
        sys.exit(1)

    accepts = payload.get("accepts") or []
    if not accepts:
        print("No accepts entry in 402 response.", file=sys.stderr)
        sys.exit(1)
    acc = accepts[0]
    resource = payload.get("resource") or {}
    url_from_resource = resource.get("url", resource_url)
    description = resource.get("description", "")

    snippet = {
        "comment": "Add to your workflow or use with rosservice call /x402_buy_service",
        "endpoint": url_from_resource or resource_url,
        "method": method,
        "payment": {
            "amount_sol": float(acc.get("amount", 0)),
            "pay_to": acc.get("payTo", ""),
            "asset": acc.get("asset", "SOL"),
            "max_timeout_seconds": acc.get("maxTimeoutSeconds", 300),
        },
        "description": description,
    }

    rosservice_example = (
        f'rosservice call /x402_buy_service "endpoint: \'{snippet["endpoint"]}\'\n'
        f'method: \'{snippet["method"]}\'\n'
        f'payload: \'{{}}\'\n'
        f'headers_json: \'{{"Content-Type": "application/json"}}\'\n'
        f'amount: {snippet["payment"]["amount_sol"]}\n'
        f'asset_symbol: \'{snippet["payment"]["asset"]}\'\n'
        f'payer_account: \'{snippet["payment"]["pay_to"]}\'"'
    )

    out = sys.stdout
    if output_path:
        try:
            out = open(output_path, "w", encoding="utf-8")
        except OSError as exc:
            print(f"Could not write {output_path}: {exc}", file=sys.stderr)
            sys.exit(1)

    try:
        out.write("# x402 Bazaar configured resource\n")
        out.write(json.dumps(snippet, indent=2) + "\n\n")
        out.write("# ROS service call example (robot pays this service):\n")
        out.write(rosservice_example + "\n")
    finally:
        if output_path and out != sys.stdout:
            out.close()

    if output_path:
        print(f"Wrote configuration to {output_path}")


def main() -> int:
    parser = argparse.ArgumentParser(
        description="x402 Bazaar CLI: search and configure paid services for robot outgoing payments."
    )
    parser.add_argument(
        "--api-url",
        default=DEFAULT_BAZAAR_API,
        help="Discovery API base URL (default: CDP Coinbase x402 discovery)",
    )
    parser.add_argument(
        "--timeout",
        type=int,
        default=15,
        help="HTTP timeout in seconds",
    )
    sub = parser.add_subparsers(dest="command", required=True)

    p_search = sub.add_parser("search", help="List resources from x402 Bazaar discovery")
    p_search.add_argument("--limit", type=int, default=20, help="Max results")
    p_search.add_argument("--offset", type=int, default=0, help="Pagination offset")

    p_show = sub.add_parser("show", help="Probe a resource URL and show 402 payment details")
    p_show.add_argument("url", help="Resource URL (e.g. https://api.example.com/v1/action)")

    p_configure = sub.add_parser(
        "configure",
        help="Generate config snippet and rosservice call template for a resource",
    )
    p_configure.add_argument("url", help="Resource URL")
    p_configure.add_argument("--method", default="POST", help="HTTP method (default: POST)")
    p_configure.add_argument("--output", "-o", help="Write to file instead of stdout")

    args = parser.parse_args()

    if args.command == "search":
        cmd_search(args.api_url, args.limit, args.offset)
    elif args.command == "show":
        cmd_show(args.url, args.timeout)
    elif args.command == "configure":
        cmd_configure(args.url, args.method, getattr(args, "output", None), args.timeout)
    else:
        parser.print_help()
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
