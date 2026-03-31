# RAID App — `POST …/teleop/help` extension (`situation_report` field)

**Full teleop cycle + SOL to operator (x402 on robot):** [RAID_APP_TELEOP_HELP_FULL_CYCLE_X402_SPEC.md](RAID_APP_TELEOP_HELP_FULL_CYCLE_X402_SPEC.md).

**Audience:** RAID App developers (`x402_raid_app` or equivalent).  
**Robot source:** package `rospy_x402`, `EscalationManager._request_grant_from_raid` → HTTP `POST` to the URL below.

## Endpoint and headers (unchanged)

- **Method:** `POST`
- **Path:** `/api/robots/{robotId}/teleop/help` (`robotId` — UUID from enroll).
- **Headers:**
  - `Content-Type: application/json`
  - `X-Robot-Teleop-Secret` — robot secret from enroll

## JSON request body

The robot sends an object like:

```json
{
  "message": "Need assistance",
  "metadata": {
    "task_id": "string",
    "error_context": "string",
    "situation_report": "string"
  }
}
```

| Field | Required | Description |
|------|----------|-------------|
| `message` | yes | Short label (as before). |
| `metadata` | yes | Object with request context. |
| `metadata.task_id` | yes | Task / session id on the robot side. |
| `metadata.error_context` | yes | String (often JSON) with machine-readable error details; may be empty. |
| `metadata.situation_report` | **new**, recommended | Free UTF-8 text: **current robot state**, **recent actions**, **why a teleoperator is needed**. May be long (thousands of chars). Old clients may omit the key — treat as `""`. |
| `metadata.kyr_peaq_context` | **new**, optional | JSON object from KYR for peaq claim binding on RAID (see `br-vr-dev-sinc/DOC/RAID_APP_PEAQ_CLAIM_SPEC.md`). If absent — do not fail. |

## RAID-side work

1. **Accept** `metadata.situation_report` in `POST …/teleop/help` (JSON parse).
2. **Persist** in the help request model and expose to operator UI/API with `task_id` / `error_context`.
3. **Backward compatibility:** if the field is missing — do not fail; treat as empty string.
4. **Limits (recommendation):** cap length at API/DB (e.g. 32–64 KiB); on overflow — `413` or truncate with log note — per product policy.
5. **Encoding:** UTF-8; do not render as raw HTML without escaping in UI.

## Related ROS API on the robot

Service `rospy_x402/RequestHelp` (`/x402/request_help`): field `situation_report` is copied to `metadata.situation_report` in the HTTP request.

Robot documentation: [RAID_INTEGRATION.md](RAID_INTEGRATION.md).
