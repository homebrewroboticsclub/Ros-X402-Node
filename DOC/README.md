# rospy_x402 — documentation index

Package documentation lives in **`DOC/`**. Root [README.md](../README.md) links here.

## REST server and code architecture

- [ARCHITECTURE.md](ARCHITECTURE.md) — nodes, rospy_x402 modules, configs, extension points.

## x402 protocol and integration

- [X402_PROTOCOL.md](X402_PROTOCOL.md) — V2: 402 response, discovery, payment verification, config format.

## Diagrams

- [ARCHITECTURE_DIAGRAMS.md](ARCHITECTURE_DIAGRAMS.md) — Mermaid: flows and verification.

## Planning / sprint (robot workspace)

- [SPRINT_STATUS_ROS_WORKSPACE.md](SPRINT_STATUS_ROS_WORKSPACE.md) — semaphores and tests for rospy_x402 vs what automated tests cover.

## Publishing

- Checklist for all four packages: [../../br_bringup/DOC/PUBLIC_RELEASE_CHECKLIST.md](../../br_bringup/DOC/PUBLIC_RELEASE_CHECKLIST.md).

## RAID App (enroll, help, allowlist)

- [RAID_INTEGRATION.md](RAID_INTEGRATION.md) — fleet secret, enroll, persistence, operator sync, `x402_ex_server` parameters.
- [RAID_APP_TELEOP_HELP_SPEC.md](RAID_APP_TELEOP_HELP_SPEC.md) — `POST …/teleop/help` contract for RAID developers (including `situation_report`).
- [RAID_APP_TELEOP_HELP_FULL_CYCLE_X402_SPEC.md](RAID_APP_TELEOP_HELP_FULL_CYCLE_X402_SPEC.md) — RAID response: signed SessionGrant, Solana `operator_pubkey`, post-payment on robot (`/x402/complete_teleop_payment`).
- [ROBOT_TELEOP_KYR_RAID_GRANT.md](ROBOT_TELEOP_KYR_RAID_GRANT.md) — RAID response order **help → Accept → GET session-grant**; robot polling in `raid_session_grant_client` + `EscalationManager`. Escalation events (help start, RAID POST, grant poll, forward to `teleop_fetch`, errors) are also written to **`~/.kyr/dashboard_events.jsonl`** for the KYR web dashboard (see `br-kyr/DOC/BLACKBOX_DASHBOARD.md`).
- [PEAQ_RAID_CLAIM.md](PEAQ_RAID_CLAIM.md) — Peaq claim via RAID: KYR context, HTTP GET, merge into dataset; specs under `br-vr-dev-sinc/DOC/`.

---

When code changes, update the matching sections; new functional area → new `DOC/` file + link here and in README.
