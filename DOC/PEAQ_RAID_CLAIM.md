# Peaq claim via RAID (robot side)

Peaq DID/claim issuance runs on **RAID** (Node.js + peaq SDK). The robot only sends KYR context in `teleop/help` and fetches `peaq_claim` over HTTP.

## Flow

1. **KYR** — service `/kyr/get_peaq_issuance_metadata` returns JSON for `metadata.kyr_peaq_context` in `POST …/teleop/help`.
2. **rospy_x402** — `EscalationManager` merges that object, then reads `peaq_claim` from the help response or polls `GET /api/robots/{robotId}/peaq/claim?helpRequestId=…` (see [RAID_APP_PEAQ_CLAIM_SPEC.md](../../br-vr-dev-sinc/DOC/RAID_APP_PEAQ_CLAIM_SPEC.md)).
3. **teleop_fetch** — service `/teleop_fetch/set_peaq_dataset_claim` writes `peaqClaim` into the dataset `metadata.json`; **dataset_upload_server** adds multipart part `peaqClaim` on push to DATA_NODE ([DATA_NODE_PEAQ_CLAIM_SPEC.md](../../br-vr-dev-sinc/DOC/DATA_NODE_PEAQ_CLAIM_SPEC.md)).

## rosparam (x402 node private namespace)

| Parameter | Default | Description |
|-----------|---------|-------------|
| `raid_peaq_claim_enabled` | `true` | If `false`, skip GET claim and dataset merge. |
| `raid_peaq_claim_timeout_sec` | `10.0` | HTTP timeout per GET. |
| `raid_peaq_claim_poll_attempts` | `3` | Retries on HTTP 404. |
| `raid_peaq_claim_poll_delay_sec` | `1.0` | Delay between polls. |

Failures are **fail-open**: teleop/help and grant forwarding still succeed if peaq steps fail.

## Code

- `src/rospy_x402/raid_peaq_client.py` — HTTP GET client.
- `src/rospy_x402/escalation_service.py` — integration.
