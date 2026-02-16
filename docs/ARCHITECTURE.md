# rospy_x402 Architecture (for developers and agents)

This document describes the current architecture of the `rospy_x402` package so that other developers or AI agents can extend, integrate, or reason about the codebase consistently.

## Overview

- **Purpose**: Expose robot capabilities over REST with pay-per-use enforcement via the x402 protocol (Solana), and allow the robot to pay external x402 services via a ROS service.
- **Stack**: ROS 1 (Noetic), Python 3, HTTP server (stdlib), Solana RPC for payment verification and sending.

## Directory layout

```
rospy_x402/
├── config/
│   └── endpoints.example.json   # Endpoint and pricing config
├── docs/                         # Architecture and protocol docs
├── scripts/
│   └── x402_ex_server.py         # ROS node: REST server + x402_buy_service
├── src/rospy_x402/
│   ├── config.py                 # ServerConfig, EndpointConfig, EndpointPricing, load_config
│   ├── server.py                 # X402RestServer (HTTP + x402 402 handling)
│   ├── health.py                 # get_health_status (used by /health endpoint)
│   ├── demo_actions.py           # Demo callables (move_demo, buy_cola_demo, shoot_demo)
│   ├── bazaar_cli.py             # Console CLI for x402 Bazaar (search/configure)
│   └── x402/                     # Reusable x402/Solana library
│       ├── client.py             # X402Client (sessions, verify_payment, send_payment)
│       ├── models.py             # PaymentRequest, PaymentSession
│       ├── schema.py             # x402 V2 response/discovery helpers
│       ├── key_manager.py        # KeyMaterial load from prompt
│       ├── solana_rpc.py         # SolanaRpcClient (getSignaturesForAddress, getTransaction)
│       ├── transaction_sender.py # SolanaTransactionSender (transfer)
│       ├── encoding.py           # (if any)
│       └── exceptions.py         # X402Error, PaymentVerificationError, etc.
├── srv/
│   └── x402_buy_service.srv      # ROS service definition
├── package.xml, setup.py, CMakeLists.txt
└── README.md
```

## Components

### 1. Configuration (`config.py`)

- **ServerConfig**: `listen_host`, `listen_port`, `require_https`, `facilitator_url`, `base_url`, `x402_network`, `endpoints`.
- **EndpointConfig** (per route): `path`, `http_method`, `description`, `ros_action` (module/callable/args/kwargs), `x402_pricing` (optional), `metadata` (e.g. `requestSchema` for Bazaar).
- **EndpointPricing**: `amount`, `asset_symbol`, `receiver_account`, `payment_window_sec`.
- Config is loaded from a single JSON file (e.g. `endpoints.example.json`). `base_url` and `x402_network` are used for x402 V2 discovery and 402 responses.

### 2. REST server (`server.py`)

- **X402RestServer**: Holds `ServerConfig`, `X402Client`, and an in-process `ThreadingHTTPServer`.
- **Routes**: Built from config; each `(path, method)` maps to one `EndpointConfig`.
- **Special route**: `GET /.well-known/x402` serves the x402 V2 discovery document (all endpoints as resources with `accepts` for paid ones).
- **Request flow**: Parse body → resolve endpoint → compute `base_url` (config or `Host` header) → `_process_endpoint_request(endpoint, body, headers, base_url)`.
- **Payment enforcement**: For endpoints with `x402_pricing`, `_ensure_payment` checks for `X-X402-Reference` (or body `x402_reference`). If missing or invalid session, it creates a payment session and raises `PaymentVerificationError` with a **x402 V2** JSON body (`x402Version: 2`, `accepts[]`, optional `resource`, `extensions.bazaar`). If session exists, it calls `X402Client.verify_payment(reference)` then proceeds.
- **Action invocation**: After payment, the handler invokes the Python callable from `ros_action` (module/callable) with optional `body`; response is `{"status": "ok", "data": result}`.

### 3. x402 library (`x402/`)

- **X402Client**: In-memory payment sessions keyed by `reference`; `create_payment_session(request, ttl_sec)`, `get_session(reference)`, `verify_payment(reference)` (on-chain check), `send_payment(destination, amount_sol)` (outgoing).
- **Payment verification**: Uses `SolanaRpcClient.get_signatures_for_address(receiver)` then for each signature `get_transaction(signature)` with `encoding: jsonParsed`, inspects `transaction.message.instructions` for `type == "transfer"`, `info.destination == receiver`, `info.lamports >= expected`; no off-chain facilitator required by default.
- **schema.py**: Helpers for x402 V2: `build_accepts_entry`, `build_402_response_v2`, `build_resource_entry`, `build_bazaar_extension`, CAIP-2 network constants for Solana.

### 4. ROS node (`scripts/x402_ex_server.py`)

- Loads config and Solana key (prompt), creates `X402RestServer` and starts it in a daemon thread.
- Advertises `x402_buy_service`: request fields include `endpoint`, `method`, `payload`, `headers_json`, `amount`, `asset_symbol`, `payer_account`. If `payer_account` is set, the node **sends** payment to that address then calls the endpoint (e.g. with `X-X402-Payment-Signature`). If empty, the node **creates an incoming payment session** and waits for the caller to pay the robot, then proceeds (used when the client pays the robot before calling).

### 5. Bazaar CLI (`bazaar_cli.py`)

- Console entry point: `x402-bazaar` (or `python -m rospy_x402.bazaar_cli`).
- **search**: Lists resources from a discovery API (default: CDP Coinbase x402 discovery).
- **show \<url\>**: Probes URL and prints 402 V2 payload if present.
- **configure \<url\>**: Fetches 402, outputs a config snippet and a `rosservice call /x402_buy_service` example for the robot to pay that service.

## Data flow summary

- **Incoming paid REST**: Client → GET/POST endpoint → 402 with V2 body → client pays to `accepts[0].payTo`, retries with `X-X402-Reference: accepts[0].extra.reference` → server verifies on-chain → executes action → 200 + JSON.
- **Outgoing (robot pays)**: Caller invokes `x402_buy_service` with `payer_account` set → node sends Solana transfer to that address → node performs HTTP request to `endpoint` with signature header → returns response to caller.
- **Discovery**: GET `/.well-known/x402` → JSON with `x402Version: 2` and `resources[]` (url, description, mimeType, accepts, extensions).

## Extension points

- Add new endpoints in the JSON config (path, method, ros_action, x402_pricing, metadata).
- Implement new callables in `demo_actions` or other modules and reference them in config.
- Use `X402Client` and `schema` helpers from non-ROS code.
- Point `--api-url` in Bazaar CLI to a custom discovery API.

All user-facing strings, logs, comments, and documentation are in English.
