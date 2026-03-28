# Architecture and x402 flow (Mermaid)

This file contains Mermaid diagrams for (A) node architecture and (B) x402 protocol implementation including payment verification. Use these as the reference for how the node works and how payments are verified.

---

## A. Node architecture (x402_ex_server)

```mermaid
flowchart TB
    subgraph ROS["ROS 1 Node (x402_ex_server)"]
        REST[X402RestServer]
        SVC[x402_buy_service]
        CFG[JSON Config]
    end

    subgraph HTTP["HTTP Layer"]
        GET["GET /.well-known/x402"]
        EP["GET/POST /path (endpoints)"]
    end

    subgraph X402Lib["rospy_x402.x402"]
        Client[X402Client]
        Schema[schema V2]
        RPC[SolanaRpcClient]
        TxSender[TransactionSender]
    end

    subgraph External["External"]
        Cli[HTTP Client]
        Solana[Solana RPC]
    end

    CFG --> REST
    REST --> GET
    REST --> EP
    GET --> Schema
    EP --> Client
    EP --> Schema
    Client --> RPC
    Client --> TxSender
    SVC --> Client
    TxSender --> Solana
    RPC --> Solana
    Cli --> GET
    Cli --> EP
```

---

## A.2 Request flow inside the node (paid endpoint)

```mermaid
sequenceDiagram
    participant Client
    participant Handler
    participant Server
    participant X402Client
    participant SolanaRPC

    Client->>Handler: GET/POST /api/v1/robot/move_demo
    Handler->>Handler: Parse path, body, Host
    Handler->>Server: _process_endpoint_request(endpoint, body, headers, base_url)

    alt No X-X402-Reference or unknown reference
        Server->>X402Client: create_payment_session(request, window_sec)
        X402Client-->>Server: PaymentSession (reference)
        Server->>Server: Build V2 402 payload (accepts, resource, extensions)
        Server-->>Handler: raise PaymentVerificationError(V2 JSON)
        Handler-->>Client: HTTP 402 + x402Version:2, accepts[], resource, extra.reference
    else Valid reference
        Server->>X402Client: verify_payment(reference)
        X402Client->>X402Client: get_session(reference)
        X402Client->>SolanaRPC: getSignaturesForAddress(receiver)
        SolanaRPC-->>X402Client: signatures[]
        loop For each signature
            X402Client->>SolanaRPC: getTransaction(sig, jsonParsed)
            SolanaRPC-->>X402Client: transaction (instructions)
            X402Client->>X402Client: Find transfer to receiver, lamports >= expected
        end
        X402Client-->>Server: PaymentSession (verified)
        Server->>Server: _invoke_action(endpoint, body)
        Server-->>Handler: { status: "ok", data }
        Handler-->>Client: HTTP 200 + JSON
    end
```

---

## B. x402 protocol implementation (overview)

```mermaid
flowchart LR
    subgraph Client["Client"]
        C1[1. Request paid endpoint]
        C2[2. Receive 402 V2]
        C3[3. Pay to payTo]
        C4[4. Retry with X-X402-Reference]
    end

    subgraph Server["Our server"]
        S1[Return 402 + accepts]
        S2[Lookup session]
        S3[Verify on-chain]
        S4[Execute action, 200]
    end

    subgraph Chain["Solana"]
        Tx[Transfer tx]
    end

    C1 --> S1
    S1 --> C2
    C2 --> C3
    C3 --> Tx
    C3 --> C4
    C4 --> S2
    S2 --> S3
    S3 --> Tx
    S3 --> S4
    S4 --> C4
```

---

## B.2 Payment verification (detail)

How we verify that the client has paid: **on-chain only** (no facilitator in the default path).

```mermaid
flowchart TD
    A[verify_payment(reference)] --> B{Session exists?}
    B -->|No| C[Raise PaymentVerificationError]
    B -->|Yes| D{Session expired?}
    D -->|Yes| E[Raise PaymentTimeoutError]
    D -->|No| F[Compute expected_lamports from session.request.amount]
    F --> G[getSignaturesForAddress(receiver, limit=200)]
    G --> H[For each signature]
    H --> I[getTransaction(signature, encoding=jsonParsed)]
    I --> J{result.meta.err?}
    J -->|Set| H
    J -->|null| K[Parse transaction.message.instructions]
    K --> L{parsed.type == "transfer" && info.destination == receiver && lamports >= expected?}
    L -->|No| H
    L -->|Yes| M[Store signature on session, return session]
```

- **Data source**: Solana RPC `getSignaturesForAddress` + `getTransaction` with `encoding: "jsonParsed"`.
- **Match criteria**: An instruction with `parsed.type === "transfer"`, `parsed.info.destination === receiver`, and `parsed.info.lamports >= expected_lamports`.
- **Commitment**: We use `"confirmed"` for both RPC calls (configurable in code if needed).
- **Caching**: Short TTL cache for signature list per receiver to avoid hammering RPC on repeated checks.

---

## B.3 Discovery and 402 response shape (V2)

```mermaid
flowchart TD
    subgraph Discovery["GET /.well-known/x402"]
        D1[x402Version: 2]
        D2[resources: array]
        D3[url, description, mimeType, accepts, extensions]
    end

    subgraph Pay["HTTP 402 response"]
        P1[x402Version: 2]
        P2[accepts: scheme, network, amount, payTo, maxTimeoutSeconds, asset, extra]
        P3[resource: url, description, mimeType]
        P4[extensions.bazaar: schema / info]
    end

    D1 --> D2
    D2 --> D3
    P1 --> P2
    P2 --> P3
    P3 --> P4
```

---

All diagrams reflect the current implementation. When changing payment verification or the x402 exchange schema, update both `X402_PROTOCOL.md` and these diagrams.
