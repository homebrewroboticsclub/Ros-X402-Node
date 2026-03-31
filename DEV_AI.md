# DEV_AI — agent context (rospy_x402)

## Ecosystem entry point (KYR + x402 + teleop)

Launch, RAID, common commands, doc index: **[../br_bringup/DEV_AI.md](../br_bringup/DEV_AI.md)**, **[../br_bringup/README.md](../br_bringup/README.md)**.

## Purpose

ROS 1 (Noetic) package: REST API over robot capabilities with **x402** (Solana) payment, ROS outgoing payment service, library `rospy_x402.x402`.

## Layout

- `.gitignore` — besides `.env` and bytecode: venv, `.pytest_cache/`, `.mypy_cache/`, IDE, `*.log`. Publishing: [../br_bringup/DOC/PUBLIC_RELEASE_CHECKLIST.md](../br_bringup/DOC/PUBLIC_RELEASE_CHECKLIST.md).
- Node: `scripts/x402_ex_server.py` → modules under `src/rospy_x402/` (`server.py`, `config.py`, `x402/*`, demo callables).
- Endpoint config: `config/endpoints.example.json`.
- Docs: [DOC/README.md](DOC/README.md). Sprint vs rospy_x402: [DOC/SPRINT_STATUS_ROS_WORKSPACE.md](DOC/SPRINT_STATUS_ROS_WORKSPACE.md).

## Responsibilities when editing

1. **Documentation** — keep `DOC/*` in sync (architecture, protocol, diagrams). Large new area (e.g. new `ros_action` type) → new `DOC/` file + links in `DOC/README.md`, [README.md](README.md), this file.
2. **Tests** — new behaviour needs tests; run full package:
   ```bash
   cd /home/ubuntu/ros_ws && source devel/setup.bash
   catkin_make run_tests --pkg rospy_x402
   ```
   If CMake has no tests — add `rostest`/`pytest` and register in `catkin_add_nosetests` or equivalent.
3. **Commit** — clear messages (what changed, API/config impact).

## Teleop: help → grant → SOL to operator

- Escalation: `POST …/teleop/help` (`EscalationManager`). Inline `teleopGrantPayload`+sig **or** poll `GET …/teleop/session-grant` (`raid_session_grant_client`, after Accept on RAID). Params: `~raid_session_grant_poll`, `~raid_session_grant_timeout_sec`, `~raid_session_grant_interval_sec`. Specs: [DOC/ROBOT_TELEOP_KYR_RAID_GRANT.md](DOC/ROBOT_TELEOP_KYR_RAID_GRANT.md), [DOC/RAID_APP_TELEOP_HELP_FULL_CYCLE_X402_SPEC.md](DOC/RAID_APP_TELEOP_HELP_FULL_CYCLE_X402_SPEC.md).
- After teleop `close_session`, `/x402/complete_teleop_payment` with `receipt_payload` — SOL to `operator_pubkey` using the same `X402Client` as `x402_buy_service`. Amount: receipt.`operator_payment_sol` (from RAID grant) → else `~teleop_operator_payment_flat_sol` (default in `ecosystem.launch` **0.0005**) → else duration × `~teleop_operator_payment_sol_per_sec`.
- Pure transfer without HTTP to target API: `rosservice call /x402_buy_service` with non-empty `payer_account` and **empty** `endpoint`.

## Peaq claim (RAID + KYR + dataset)

- Robot sends `metadata.kyr_peaq_context` in `teleop/help`, fetches `peaq_claim` (inline or `GET …/peaq/claim`), calls `/teleop_fetch/set_peaq_dataset_claim`. Details: [DOC/PEAQ_RAID_CLAIM.md](DOC/PEAQ_RAID_CLAIM.md). RAID/DATA_NODE specs: `br-vr-dev-sinc/DOC/RAID_APP_PEAQ_CLAIM_SPEC.md`, `DATA_NODE_PEAQ_CLAIM_SPEC.md`.
- No claim (including `claim_not_ready` on Peaq/RAID) — **fail-open**: help and grant still work; see “Operational status” in `PEAQ_RAID_CLAIM.md`.
- Build/runtime dependency: package **KYR** (`GetPeaqIssuanceMetadata`).

## Useful links

- [DOC/ARCHITECTURE.md](DOC/ARCHITECTURE.md)
- [DOC/X402_PROTOCOL.md](DOC/X402_PROTOCOL.md)
- [DOC/PEAQ_RAID_CLAIM.md](DOC/PEAQ_RAID_CLAIM.md)
