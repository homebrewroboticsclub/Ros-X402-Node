# Robot teleop integration with RAID: SessionGrant, KYR, operator payment

**Audience:** robot software developers (`teleop_fetch`, `rospy_x402`, KYR, `EscalationManager`).  
**RAID side:** `x402_raid_app` repository.  
**Related:** [RAID_APP_TELEOP_HELP_FULL_CYCLE_X402_SPEC.md](RAID_APP_TELEOP_HELP_FULL_CYCLE_X402_SPEC.md), [TELEOP_FETCH.md](TELEOP_FETCH.md).

**Robot implementation:** after `POST …/teleop/help`, if the response has no inline `teleopGrantPayload`, `EscalationManager` calls **`raid_session_grant_client.poll_raid_session_grant`** (params `~raid_session_grant_poll`, `~raid_session_grant_timeout_sec`, `~raid_session_grant_interval_sec` on `x402_server`).

---

## 1. Symptoms on the robot

In logs after teleop session end:

- `complete_teleop_payment: success but NO on-chain transfer`
- `No valid operator Solana pubkey in receipt`
- mention of **`pending_from_raid`**

**Meaning:** **SignedReceipt** from KYR has no valid Solana operator address. It should come from a **signed SessionGrant** issued by RAID, not the local mock.

---

## 2. What RAID does and does not do

| Action | Where |
|--------|--------|
| Accepts help request | `POST /api/robots/{robotId}/teleop/help` + robot secret |
| After operator **accept**, signs SessionGrant with **`operator_pubkey`** (operator wallet from RAID DB) | DB row + `GET …/teleop/session-grant` |
| SOL transfer to operator | **Does not.** **Robot** pays via `/x402/complete_teleop_payment` and its payer |

If KYR opened with a **mock grant** (`operator_pubkey: pending_from_raid`), the receipt has no real address — this is a **step-order or trust bug on the robot**, not “stale data” in the first `POST …/help`.

---

## 3. Required step order (robot)

```text
1) POST …/teleop/help
      → store helpRequest.id (and teleopGrantPollUrl from response if present)

2) Wait until operator presses Accept in RAID (human/UI).

3) GET …/teleop/session-grant?helpRequestId=<uuid from step 1>
      → same robot secret header as help
      → 200: teleopGrantPayload (JSON string), teleopGrantSignature (base58), grantSignerPublicKey

4) Parse teleopGrantPayload (UTF-8, byte-for-byte string — option A in spec).
      → check: operator_pubkey is valid Solana base58, NOT "pending_from_raid"

5) Pass payload + signature to KYR open_session (receive_grant / equivalent).
      → trust grant signer public key: grantSignerPublicKey (see §5)

6) Teleop session → close_session → complete_teleop_payment with KYR receipt
```

**Invalid for payment:** KYR `open_session` with **internal mock grant** only, then never replacing with RAID data.

---

## 4. HTTP contract (robot → RAID)

### 4.1 Secret

Header: **`X-Robot-Teleop-Secret: <teleopSecret>`**  
(or `Authorization: Bearer <teleopSecret>` — as agreed with RAID.)

**`teleopSecret`** is issued when the robot registers on RAID (enroll / admin).

### 4.2 Help request

```http
POST /api/robots/{robotId}/teleop/help
X-Robot-Teleop-Secret: <secret>
Content-Type: application/json

{ "message": "…", "metadata": { "task_id": "…", … } }
```

In the response (when grant signing is enabled on RAID):

- **`id`** / **`helpRequest.id`** — request UUID; **required** for step 3.
- **`teleopGrantPollUrl`** — relative path, e.g.  
  `/api/robots/{robotId}/teleop/session-grant?helpRequestId={id}`  
  (prepend RAID scheme and host, e.g. `https://raid.example`).

### 4.3 Fetch grant (only after operator accept)

```http
GET /api/robots/{robotId}/teleop/session-grant?helpRequestId=<uuid>
X-Robot-Teleop-Secret: <secret>
```

**Success 200** (example fields):

| Field | Type | Description |
|------|------|-------------|
| `teleopGrantPayload` | string | Exact UTF-8 JSON SessionGrant; **do not re-serialize** for KYR signature check |
| `teleopGrantSignature` | string | Ed25519 signature **base58** over **raw UTF-8 bytes** of `teleopGrantPayload` |
| `grantSignerPublicKey` | string | Solana base58 **grant signer** (RAID). Must be in KYR `trusted_raid_keys` |

**404 errors (JSON body, `error` field):**

| error | When |
|-------|------|
| `grant_not_ready` | Request is still **open** (no operator accept yet), **or** the help request was returned to **open** and grant fields were **cleared** (e.g. operator **`decline-before-connect`** after accept but before **`/ws/teleop/session/{sessionId}`**). **Also:** if your poller previously received **200** with a grant, then sees **`grant_not_ready`**, you must **invalidate any cached `teleopGrantPayload` / signature** and keep polling until a **fresh 200** — the old `session_id` in the discarded grant must not be used for KYR. |
| `grant_unconfigured` | RAID missing `TELEOP_GRANT_SIGNING_SECRET_KEY` |
| `grant_absent` | Operator has no `wallet_public_key` in RAID DB |

Robot strategy: after help, **poll** `session-grant` with backoff until 200 or product timeout. Treat **`grant_not_ready` after a prior 200** as **grant revoked** until the next **200**.

### 4.4 `poll_raid_session_grant` and cache / race notes

**Shipped robot behavior (`rospy_x402`):** `poll_raid_session_grant` runs in a **single synchronous loop** and returns on the **first** **200** that includes a valid `teleopGrantPayload` + signature pair. It does **not** keep polling after success, so it does **not** retain a stale grant across subsequent GETs in that call.

**Stateful pollers or custom caches:** if your integration **stores** payload/signature after a **200**, you **must** drop that cache when a later GET returns **`grant_not_ready`** (see table above).

**Race after poll returns:** there is a small window between **return from `poll_raid_session_grant`** and **`/teleop_fetch/receive_grant` → KYR `open_session`** where the operator could **`decline-before-connect`** on RAID. The grant may still verify cryptographically until expiry while RAID has already cleared its row — product risk is **low**; tightening would require an extra **`GET …/session-grant`** immediately before `open_session` (not implemented by default).

---

## 5. Trust on KYR (`trusted_raid_keys`)

The grant is signed by a **separate** RAID Ed25519 / Solana key, **not** the operator wallet.

KYR must allow a public key matching:

- **`grantSignerPublicKey`** from `session-grant`, or
- **`teleopGrantSignerPublicKey`** from **`GET /health`** on the same RAID instance.

If the key is missing or wrong, KYR may reject the grant and stay on mock → receipt again has **`pending_from_raid`**.

---

## 6. SessionGrant contents (after `JSON.parse(teleopGrantPayload)`)

Expected fields (**JSON names from RAID**):

| Field | Description |
|------|-------------|
| `session_id` | Teleop proxy session UUID in RAID (**`session.id`** in operator accept response). Should match how KYR/teleop_fetch bind the session |
| `robot_id` | Robot UUID on RAID |
| `task_id` | From `metadata.task_id` on help (may be empty) |
| `operator_pubkey` | **Solana base58** SOL recipient |
| `valid_until_sec` | Grant expiry Unix time |
| `scope_json` | JSON string (policy). RAID may add **`teleop_payment_mode` / `teleop_operator_flat_sol`** hints for other consumers; **`rospy_x402` does not parse them** — use **`operator_payment_sol`** on the grant or rosparams for payout (see [RAID_APP_TELEOP_HELP_FULL_CYCLE_X402_SPEC.md](RAID_APP_TELEOP_HELP_FULL_CYCLE_X402_SPEC.md) §2.4). |

If **`operator_pubkey`** is missing or equals the mock placeholder — do **not** expect on-chain operator payment from `complete_teleop_payment`.

---

## 7. Self-check from a workstation

Set `RAID_BASE`, `robotId`, `secret`, `helpRequestId` (after accept):

```bash
curl -sS -H "X-Robot-Teleop-Secret: ${SECRET}" \
  "${RAID_BASE}/api/robots/${ROBOT_ID}/teleop/session-grant?helpRequestId=${HELP_ID}"
```

Then:

```bash
curl -sS -H "X-Robot-Teleop-Secret: ${SECRET}" \
  "${RAID_BASE}/api/robots/${ROBOT_ID}/teleop/session-grant?helpRequestId=${HELP_ID}" \
  | jq -r '.teleopGrantPayload | fromjson | .operator_pubkey'
```

Expect non-empty base58, not `pending_from_raid`.

---

## 8. Checklist before escalating to RAID team

- [ ] After **accept**, `GET session-grant` returns **200** with non-empty `teleopGrantPayload` and `teleopGrantSignature`.
- [ ] Poller **drops cached grant** when **`grant_not_ready`** appears after a prior **200** (operator may have **decline-before-connect**).
- [ ] Parsed payload **`operator_pubkey`** is valid Solana.
- [ ] KYR **`trusted_raid_keys`** includes **`grantSignerPublicKey`** from RAID.
- [ ] KYR **`open_session`** runs **after** receiving this grant with **these** payload/signature, not mock-only.
- [ ] Robot has RPC, payer balance, and **`/x402/complete_teleop_payment`** configured (outside RAID).

If all hold and still no on-chain transfer — inspect Solana/x402 logs on the **robot** (RPC error, amount, tx signature).

---

## 9. Links

- Full cycle and diagram: [RAID_APP_TELEOP_HELP_FULL_CYCLE_X402_SPEC.md](RAID_APP_TELEOP_HELP_FULL_CYCLE_X402_SPEC.md)  
- HTTP help from robot: [TELEOP_FETCH.md](TELEOP_FETCH.md)  
- RAID OpenAPI: `https://<raid-host>/docs`
