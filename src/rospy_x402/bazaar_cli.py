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
import urllib.parse
import urllib.request
from typing import Any, Dict, List, Optional, Union


DEFAULT_BAZAAR_API = "https://api.cdp.coinbase.com/platform/v2/x402/discovery/resources"


def _resource_matches_filter(r: Dict[str, Any], filter_str: str) -> bool:
    """True if filter_str (case-insensitive) appears in description, resource URL, or metadata."""
    if not filter_str:
        return True
    needle = filter_str.lower()
    url_val = (r.get("resource") or r.get("url") or "").lower()
    if needle in url_val:
        return True
    meta = r.get("metadata") or {}
    for key in ("description", "name", "category", "tags", "title"):
        val = meta.get(key)
        if isinstance(val, str) and needle in val.lower():
            return True
        if isinstance(val, list) and any(needle in (str(v).lower()) for v in val):
            return True
    for acc in r.get("accepts") or []:
        if isinstance(acc, dict) and needle in (acc.get("description") or "").lower():
            return True
    return False


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


def cmd_search(
    api_url: str,
    limit: int,
    offset: int,
    query: Optional[str] = None,
    filter_str: Optional[str] = None,
    api_type: Optional[str] = None,
    api_network: Optional[str] = None,
    show_index: Optional[int] = None,
    configure_index: Optional[int] = None,
    configure_output: Optional[str] = None,
    timeout: int = 15,
) -> None:
    """List resources from the x402 Bazaar discovery API."""
    params = []
    if limit > 0:
        params.append(f"limit={limit}")
    if offset > 0:
        params.append(f"offset={offset}")
    if query:
        params.append(f"query={urllib.parse.quote(query)}")
    if api_type:
        params.append(f"type={urllib.parse.quote(api_type)}")
    if api_network:
        params.append(f"network={urllib.parse.quote(api_network)}")
    url = api_url + ("?" + "&".join(params) if params else "")
    try:
        data = _http_get(url)
    except (urllib.error.URLError, urllib.error.HTTPError, json.JSONDecodeError) as exc:
        print(f"Error fetching discovery: {exc}", file=sys.stderr)
        sys.exit(1)

    # CDP API uses "items" + "pagination"; support legacy "resources" + top-level total/limit/offset
    raw = data.get("items") or data.get("resources")
    resources: List[Dict[str, Any]] = raw if isinstance(raw, list) else []
    if filter_str:
        resources = [r for r in resources if _resource_matches_filter(r, filter_str)]
        # Show that we applied a client-side filter
        if filter_str and (data.get("items") or data.get("resources")):
            total_fetched = len(data.get("items") or data.get("resources") or [])
            print(f"Filter: \"{filter_str}\" (matched {len(resources)} of {total_fetched} on this page)\n")
    pagination = data.get("pagination") or {}
    total = pagination.get("total") or data.get("total", len(resources))
    limit_val = pagination.get("limit", data.get("limit", limit))
    offset_val = pagination.get("offset", data.get("offset", offset))
    print(f"Resources (total={total}, limit={limit_val}, offset={offset_val}):\n")
    for i, r in enumerate(resources, start=1):
        url_val = r.get("resource") or r.get("url", "")
        meta = r.get("metadata") or {}
        desc = (meta.get("description") or "")
        if not desc and r.get("accepts"):
            acc = r["accepts"][0] if isinstance(r["accepts"], list) else r["accepts"]
            desc = (acc.get("description") or "") if isinstance(acc, dict) else ""
        desc = desc[:60]
        rtype = r.get("type", "http")
        print(f"  {i}. {url_val}")
        print(f"     type={rtype}  {desc}")
    if not resources:
        print("  (none)")
    else:
        print("\n  Show details:    x402-bazaar show <url>")
        print("  Generate config: x402-bazaar configure <url> -o out.json")
        print("  Shortcuts:       search --show-index N   or   search --configure-index N -o out.json")

    # Shortcuts: run show or configure on Nth result (1-based index)
    idx = show_index or configure_index
    if idx is not None and resources:
        one_based = max(1, int(idx))
        if one_based > len(resources):
            print(f"\nNo resource at index {one_based} (max {len(resources)}).", file=sys.stderr)
            sys.exit(1)
        chosen = resources[one_based - 1]
        chosen_url = chosen.get("resource") or chosen.get("url", "")
        if show_index is not None:
            print(f"\n--- show #{one_based} ---\n")
            cmd_show(chosen_url, timeout)
        else:
            print(f"\n--- configure #{one_based} ---\n")
            cmd_configure(chosen_url, "POST", configure_output, timeout)


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
    p_search.add_argument("--limit", "-n", type=int, default=20, help="Max results (default: 20)")
    p_search.add_argument("--offset", type=int, default=0, help="Pagination offset")
    p_search.add_argument(
        "--query", "-q",
        type=str,
        default=None,
        help="Search query (passed to discovery API if supported)",
    )
    p_search.add_argument(
        "--filter", "-f",
        dest="filter_str",
        type=str,
        default=None,
        metavar="TEXT",
        help="Client-side filter: keep only resources whose description, URL, or metadata contain TEXT (e.g. LLM, sentiment, storage). Applied to current page.",
    )
    p_search.add_argument(
        "--type",
        dest="api_type",
        type=str,
        default=None,
        metavar="TYPE",
        help="Pass type to discovery API if supported (e.g. http).",
    )
    p_search.add_argument(
        "--network",
        dest="api_network",
        type=str,
        default=None,
        metavar="NETWORK",
        help="Pass network to discovery API if supported (e.g. base, solana).",
    )
    p_search.add_argument(
        "--show-index", "-s",
        type=int,
        default=None,
        metavar="N",
        help="After listing, show 402 details for result N (1-based)",
    )
    p_search.add_argument(
        "--configure-index", "-c",
        type=int,
        default=None,
        metavar="N",
        help="After listing, generate config for result N (1-based); use -o to write file",
    )
    p_search.add_argument(
        "--output", "-o",
        dest="configure_output",
        help="With --configure-index: write config to this file",
    )

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
        cmd_search(
            args.api_url,
            args.limit,
            args.offset,
            query=getattr(args, "query", None),
            filter_str=getattr(args, "filter_str", None),
            api_type=getattr(args, "api_type", None),
            api_network=getattr(args, "api_network", None),
            show_index=getattr(args, "show_index", None),
            configure_index=getattr(args, "configure_index", None),
            configure_output=getattr(args, "configure_output", None),
            timeout=args.timeout,
        )
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
