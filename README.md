# rospy_x402

`rospy_x402` provides a ROS 1 (Noetic) integration that exposes robot capabilities over a configurable REST API and enforces access through the Solana-based [x402](https://www.x402.org/) payment protocol. It includes:

- A REST server node (`x402_ex_server`) that loads endpoints from JSON configuration and requires successful x402 payments before invoking robot actions.
- A `x402_buy_service` ROS service that performs paid calls to external HTTP services, supporting both incoming and outgoing Solana payments.
- A reusable `rospy_x402.x402` Python library that encapsulates Solana RPC interactions, transaction submission, and payment session validation.

All runtime logs and comments are in English, while the ROS package name follows the project requirements.

## Features

- **Configurable HTTP endpoints**: Define REST routes, link them to Python callables, and attach payment requirements through JSON configs.
- **Payment enforcement**: Automatically create payment sessions, monitor Solana blockchain transfers, and require valid signatures for execution.
- **Outgoing payments**: Allow the robot to pay external parties directly via Solana when requested through the ROS service.
- **In-memory key handling**: The node prompts for a Solana private key on startup and keeps it exclusively in RAM.
- **Library-first architecture**: All x402/Solana logic is separated into clean modules for reuse in other projects.

## Prerequisites

- ROS 1 Noetic with a properly initialized catkin workspace (`/home/ubuntu/ros_ws` in the examples below).
- Python dependencies:
  - `python3-nacl` (installed via `apt` or `pip`).
  - `solana` (installed via `pip`) for outgoing payments. If absent, the node can still receive payments but cannot send them.
- Network access to a Solana RPC endpoint (default: `https://api.mainnet-beta.solana.com`). You can use a custom endpoint or a private RPC Gateway if required.

### Installing Python dependencies

```bash
sudo apt-get install python3-nacl
pip3 install --user solana
```

Add `~/.local/bin` to your `PATH` if you use the `--user` flag for `pip`.

## Building the Package

From the workspace root:

```bash
cd /home/ubuntu/ros_ws
catkin build rospy_x402
source devel/setup.bash
```

The package depends on `message_generation`, so ensure your workspace is sourced before building to pick up generated service headers.

## Launching the REST Server Node

```bash
rosrun rospy_x402 x402_ex_server.py _config_path:=/path/to/endpoints.json _solana_rpc_endpoint:=https://api.mainnet-beta.solana.com
```

At startup the node prompts for a **base58-encoded Solana private key**. The key is kept only in memory and is used to:

- Derive the default receiver account for incoming payments.
- Sign transactions if the node must initiate outgoing payments.

If you do not pass the `~service_receiver_account` parameter, the derived public key is used by default.

### Important Parameters

- `~config_path` *(string)*: Path to the JSON configuration that defines REST endpoints. Defaults to the bundled `config/endpoints.example.json`.
- `~solana_rpc_endpoint` *(string)*: Solana RPC URL used for payment verification and transaction submission.
- `~service_receiver_account` *(string)*: Optional explicit Solana address to receive payments. Defaults to the public key derived from the private key.
- `~service_asset_symbol` *(string)*: Asset ticker (default `SOL`).
- `~service_payment_window_sec` *(int)*: Number of seconds to wait for an incoming payment (default `300`).
- `~service_poll_interval_sec` *(float)*: Polling frequency (seconds) for payment confirmation (default `5.0`).

### Configuration File Format

`config/endpoints.example.json` illustrates a single POST endpoint:

```json
{
  "listen_host": "0.0.0.0",
  "listen_port": 18080,
  "require_https": false,
  "endpoints": [
    {
      "path": "/health",
      "http_method": "GET",
      "description": "External integration health check.",
      "ros_action": {
        "type": "python_call",
        "module": "rospy_x402.health",
        "callable": "get_health_status"
      }
    },
    {
      "path": "/api/v1/robot/move_demo",
      "http_method": "POST",
      "description": "Trigger a sample motion via MotionManager for demonstration purposes.",
      "ros_action": {
        "type": "python_call",
        "module": "rospy_x402.demo_actions",
        "callable": "move_demo"
      },
      "x402_pricing": {
        "amount": 0.00005,
        "asset_symbol": "SOL",
        "receiver_account": "",
        "payment_window_sec": 180
      }
    }
  ]
}
```

- `ros_action`: Defines how to invoke the robot functionality. The example calls `rospy_x402.demo_actions.move_demo`, which internally uses `MotionManager` and defaults to the `wave` action group (you can override it by sending `{"demo_name": "<your_action>"}` in the request body).
- `x402_pricing`: Enables payment enforcement. `receiver_account` can be omitted to use the node’s default account.
- `facilitator_url`: Optional URL of an external facilitator service; omit it to rely on the built-in on-chain verification.
- Omit `x402_pricing` entirely to expose a free endpoint.

### How payment verification works

1. When a client first hits a paid endpoint without a valid reference, the server calls `X402Client.create_payment_session()` and returns HTTP 402 with a JSON payload containing `reference`, `amount`, `asset`, `receiver`, and `expires_in_sec`.
2. The client transfers the requested amount (on Solana by default) and retries the call, now including the `X-X402-Reference` header (or `x402_reference` field in the body).
3. The server invokes `X402Client.verify_payment()`:
   - Fetches the latest confirmed signatures for the receiver address.
   - Retrieves each transaction, scanning for a `transfer` instruction whose `destination` matches the receiver and whose lamports exceed the expected amount.
   - On success, stores the Solana signature in the payment session so it can be returned in responses or logs.
4. If the transfer is missing or the session expired, the client gets another HTTP 402 with a descriptive error message. Otherwise the requested ROS action executes and the HTTP response is `200 OK`.

Because verification uses confirmed Solana transactions, the process is non-custodial and does not rely on any off-chain authority.

> **Note:** In the example configuration the `receiver_account` fields are left blank. At runtime the server fills them with the public key derived from the private key you enter on startup (or the value provided via `~service_receiver_account`). Replace them with explicit addresses only if you need to override that behavior.

#### Using an external facilitator (optional)

Some deployments prefer delegating payment tracking to a third-party service (“facilitator”) that aggregates metrics, provides fiat ramps, or proxies blockchain queries. By default the server performs on-chain verification itself—no facilitator is used unless you explicitly define one. To opt in, add the following key at the root of your configuration file:

```json
"facilitator_url": "https://facilitator.example.com/api/v1/payments"
```

The x402 server surfaces the configured URL (via `X402RestServer.facilitator_url`) so integrators can:

- Redirect payment verification to the facilitator,
- mirror ledger data for auditing dashboards,
- or replace the default Solana RPC polling logic with service-specific hooks.

If `facilitator_url` is omitted, the node continues to verify payments directly against the Solana RPC endpoint configured at startup.

## Example Usage

The snippets below assume you are on Solana **Devnet**. Replace URLs, keys, and amounts to match your environment.

### 1. Prepare and start the node

```bash
solana config set --url https://api.devnet.solana.com
solana-keygen new --outfile ~/devnet.json
solana airdrop 2 "$(solana-keygen pubkey ~/devnet.json)"

source /opt/ros/noetic/setup.bash
source /home/ubuntu/ros_ws/devel/setup.bash
rosrun rospy_x402 x402_ex_server.py \
  _config_path:=/home/ubuntu/ros_ws/src/rospy_x402/config/endpoints.example.json \
  _solana_rpc_endpoint:=https://api.devnet.solana.com
```

When prompted, paste the base58-encoded private key from `~/devnet.json`. The node logs the derived public key:

```
[INFO] Loaded x402 key. Public key: <YOUR_PUBLIC_KEY>
```

### 2. Test a paid REST endpoint (incoming payment)

1. Send an HTTP request without paying:
   ```bash
   curl -X POST http://localhost:18080/api/v1/robot/move_demo \
     -H 'Content-Type: application/json' \
     -d '{}'
   ```
   The response has HTTP 402 and JSON similar to:
   ```json
   {
     "message": "Payment session created. Complete payment to continue.",
    "reference": "bf9a6f7b4bbd4cbab226f0ddc5cf96d1",
    "receiver": "<YOUR_PUBLIC_KEY>",
    "amount": 0.00005,
     "asset": "SOL",
     "expires_in_sec": 180
   }
   ```

2. Pay the required amount (0.00005 SOL) to the listed `receiver` address:
   ```bash
   solana transfer <YOUR_PUBLIC_KEY> 0.00005 --allow-unfunded-recipient
   ```

3. Repeat the HTTP request; after the transfer is confirmed, the response becomes:
   ```json
   {
     "status": "ok",
     "data": {
       "status": "completed",
       "action": "wave"
     }
   }
   ```

### 3. Integrate with the public health endpoint

```bash
curl -s http://localhost:18080/health | jq
```

Sample response (values trimmed for brevity):

```json
{
  "status": "ready",
  "message": "Ready for commands",
  "availableMethods": [
    {
      "path": "/health",
      "httpMethod": "GET",
      "description": "External integration health check.",
      "rosAction": {
        "module": "rospy_x402.health",
        "callable": "get_health_status"
      },
      "parameters": {},
      "pricing": null
    },
    {
      "path": "/api/v1/robot/move_demo",
      "httpMethod": "POST",
      "description": "Trigger a sample motion via MotionManager for demonstration purposes.",
      "rosAction": {
        "module": "rospy_x402.demo_actions",
        "callable": "move_demo"
      },
      "parameters": {},
      "pricing": {
        "amount": 0.00005,
        "assetSymbol": "SOL",
        "receiverAccount": "<derived from private key>",
        "paymentWindowSec": 180
      },
      "metadata": {
        "requestSchema": {
          "type": "object",
          "description": "Optional payload allowing you to pick a custom action group.",
          "required": [],
          "properties": {
            "demo_name": {
              "type": "string",
              "description": "Action group name passed to MotionManager.run_action (defaults to 'wave')."
            }
          }
        }
      }
    },
    {
      "path": "/api/v1/robot/buy_cola_demo",
      "httpMethod": "POST",
      "description": "Paid demo endpoint that drives the robot forward for 5 seconds to deliver cola cans.",
      "rosAction": {
        "module": "rospy_x402.demo_actions",
        "callable": "buy_cola_demo"
      },
      "parameters": {},
      "pricing": {
        "amount": 0.000075,
        "assetSymbol": "SOL",
        "receiverAccount": "<derived from private key>",
        "paymentWindowSec": 240
      },
      "metadata": {
        "requestSchema": {
          "type": "object",
          "required": [
            "quantity",
            "buyer",
            "seller"
          ],
          "properties": {
            "quantity": {
              "type": "integer",
              "minimum": 1,
              "description": "Number of cola cans to purchase."
            },
            "buyer": {
              "type": "object",
              "required": [
                "lat",
                "lng"
              ],
              "properties": {
                "lat": {
                  "type": "number"
                },
                "lng": {
                  "type": "number"
                }
              },
              "description": "Latitude/longitude of the buyer."
            },
            "seller": {
              "type": "object",
              "required": [
                "lat",
                "lng"
              ],
              "properties": {
                "lat": {
                  "type": "number"
                },
                "lng": {
                  "type": "number"
                }
              },
              "description": "Latitude/longitude of the seller."
            }
          }
        }
      }
    },
    {
      "path": "/api/v1/robot/shoot_demo",
      "httpMethod": "POST",
      "description": "Paid endpoint that fires the robot fire control system once.",
      "rosAction": {
        "module": "rospy_x402.demo_actions",
        "callable": "shoot_demo"
      },
      "parameters": {},
      "pricing": {
        "amount": 0.001,
        "assetSymbol": "SOL",
        "receiverAccount": "<derived from private key>",
        "paymentWindowSec": 180
      },
      "metadata": {
        "requestSchema": {
          "type": "object",
          "description": "No payload required; call with an empty JSON object to fire once.",
          "required": [],
          "properties": {}
        }
      }
    }
  ],
  "location": {
    "lat": 22.298412,
    "lng": 114.167582
  }
}
```

`availableMethods` now reflects the live endpoint configuration (path, HTTP method, pricing, default arguments). Coordinates are still mocked randomly around Hong Kong—update `rospy_x402.health.get_health_status()` to subscribe to a GPS topic and feed real values to integrators.

### 4. Try the cola purchase demo endpoint

The cola demo requires a slightly higher payment (0.000075 SOL) and validates the payload:

1. Initiate the request:
   ```bash
   curl -X POST http://localhost:18080/api/v1/robot/buy_cola_demo \
     -H 'Content-Type: application/json' \
     -d '{
       "quantity": 2,
       "buyer": {"lat": 55.7558, "lng": 37.6173},
       "seller": {"lat": 55.7562, "lng": 37.6181}
     }'
   ```
   Until the payment is settled, the response is HTTP 402 with the payment session description (similar to the `move_demo` example above) but with `amount: 0.000075`.

2. After transferring 0.000075 SOL to the provided receiver and the transaction is confirmed, repeat the request. The success payload resembles:
   ```json
   {
     "status": "ok",
     "data": {
       "order": {
         "item": "cola_can",
         "quantity": 2
       },
       "buyer": {
         "lat": 55.7558,
         "lng": 37.6173
       },
       "seller": {
         "lat": 55.7562,
         "lng": 37.6181
       },
       "movement": {
         "type": "forward",
         "durationSec": 5,
         "velocityPreset": 2,
         "stepAmplitude": 0.02
       },
       "status": "completed"
     }
   }
   ```

### 5. Fire the shoot demo endpoint

The shoot demo publishes a single `"fire"` command to the `firecontroll` topic. It requires a larger payment of **0.001 SOL**.

```bash
curl -X POST http://localhost:18080/api/v1/robot/shoot_demo \
  -H 'Content-Type: application/json' \
  -d '{}'
```

As with the other paid endpoints, the first call returns HTTP 402 with a payment session description (this time with `amount: 0.001`). After transferring the funds, repeat the request with the `X-X402-Reference` header to receive:

```json
{
  "status": "ok",
  "data": {
    "status": "fired",
    "topic": "firecontroll",
    "payload": "fire"
  }
}
```

### 6. Call the ROS service (outgoing payment)

```bash
rosservice call /x402_buy_service "endpoint: 'https://httpbin.org/post'
method: 'POST'
payload: '{\"demo\": true}'
headers_json: '{\"Content-Type\": \"application/json\"}'
amount: 0.00005
asset_symbol: 'SOL'
payer_account: '<RECIPIENT_PUBLIC_KEY>'"
```

Because `payer_account` is provided, the node pays the remote party and attaches the Solana transaction signature to the response:

```yaml
success: True
response_body: "{\n  \"json\": {\n    \"demo\": true\n  },\n  \"url\": \"https://httpbin.org/post\"\n}"
error_message: ''
payment_signature: '5Z...abc'
```

If `payer_account` is left empty, the service behaves like the REST endpoint: it creates a payment session and waits for the caller to transfer funds to the robot’s account.

## ROS Service `x402_buy_service`

Service definition (`srv/x402_buy_service.srv`):

| Field | Type | Description |
|-------|------|-------------|
| `endpoint` | `string` | URL or IP of external service to call |
| `method` | `string` | HTTP method (GET, POST, …) |
| `payload` | `string` | Raw request body |
| `headers_json` | `string` | JSON object with custom headers |
| `amount` | `float64` | Payment amount in SOL (or selected asset) |
| `asset_symbol` | `string` | Asset ticker (defaults to `SOL`) |
| `payer_account` | `string` | Optional Solana address to pay (robot sends funds if set) |

Response fields:

- `success` *(bool)*: Indicates whether payment and external call succeeded.
- `response_body` *(string)*: Raw response from the external service (on success).
- `error_message` *(string)*: Explanation of failure.
- `payment_signature` *(string)*: Solana transaction signature (incoming or outgoing payment).

### Usage Pattern

- **Incoming payment** (`payer_account` empty): The service creates a payment session and waits for the caller to transfer funds. Provide the session reference (returned in error payload) to your client so it can pay.
- **Outgoing payment** (`payer_account` set): The node immediately sends funds to the provided address using the loaded private key. This requires the `solana` Python package and sufficient balance.

## Security Notes

- The private key is requested interactively through `getpass` and stored only in memory. Restarting the node requires re-entering the key.
- Keep the JSON configuration files secure. They control which Python callables can be invoked via REST.
- Always use HTTPS (reverse proxy or VPN) when exposing the REST API over untrusted networks.

## Development Hints

- The x402/Solana logic is located under `rospy_x402/src/rospy_x402/x402/`. You can reuse these modules for non-ROS projects.
- Extend `config/endpoints.example.json` into your own configuration, then pass its path via `~config_path`.
- For unit testing, consider mocking Solana RPC responses and transaction confirmations.

## Troubleshooting

- **Missing `solana` package**: Outgoing payments raise `PaymentSubmissionError`. Install `solana` via `pip`.
- **Payment verification fails**: Ensure the RPC endpoint is reachable and returns confirmed transactions. Increase `payment_window_sec` or adjust `poll_interval_sec` if the network is slow.
- **catkin build failures**: Verify `message_generation`, `message_runtime`, and Python dependencies are installed, then re-source `devel/setup.bash`.
