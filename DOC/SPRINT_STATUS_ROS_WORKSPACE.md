# Sprint: semaphores and tests — **rospy_x402** ownership

Status as of **2026-03-31**. This package covers **ROS x402 integration, RAID HTTP, teleop escalation, post-operator SOL payment**, and extra logging to `~/.kyr/dashboard_events.jsonl`. Cloud receipts/incidents tables, Hugging Face, quality score, external incident DB — **out of repo**.

## Summary

| Category | Comment |
|----------|---------|
| Implemented | `EscalationManager`: `POST …/teleop/help`, `session-grant` poll, grant handoff to `teleop_fetch`, optional Peaq claim → `set_peaq_dataset_claim`, dashboard log events; RAID clients, escalation tests |
| Partial | Item 7 (`raid_task_id` / `payment_id` in SessionRecord): robot has `task_id` in grant/event and x402 chain; **sprint-named fields in cloud SessionRecord** — Backend/DATA_NODE |
| Not in repo | REST `GET /api/v1/receipts`, receipt↔incident link, auto `TELEOP_TAKEOVER`, incidents, RTT gate, Cosmos, recovery extractor, HF, 500+ external incidents |

---

## Semaphores → rospy_x402

| # | Deliverable | Package role | Status |
|---|-------------|--------------|--------|
| 1 | Receipt emission service | x402 REST — **payments/robot demo**, not cloud receipt CRUD | **N/A** as HTTP receipts API; overlap with KYR receipt via `close_session` / payment only |
| 2 | Receipt–incident | None | **N/A** |
| 3 | Event↔recording | Indirect: escalation carries `task_id`, dataset in teleop_fetch | **Partial** as downstream data only |
| 4 | `TELEOP_TAKEOVER` auto | No such cloud event from rospy_x402 | **Not implemented** |
| 5 | Incident auto-generation | None | **N/A** |
| 6 | RTT preflight | None | **N/A** |
| 7 | `raid_task_id` + `payment_id` | Grant/RAID context; Peaq merge in metadata — see `escalation_service`, `PEAQ_RAID_CLAIM.md` | **🟡 PARTIAL**: robot/RAID; cloud SessionRecord — out of repo |
| 8 | KYR stats | No cloud endpoint | **N/A** |
| 9–11 | Handoff / VR | None | **N/A** |
| 17 | peaq ClaimRegistry real calls | HTTP GET peaq claim, proxy to dataset | **🟡**: **robot side** done; `claim_id` in cloud SessionRecord — Backend |

Other items (12–16, 18–21, 22 tests 12–24) — **not rospy_x402**.

---

## Sprint tests → rospy_x402

| # | Test | Status |
|---|------|--------|
| 1–6, 8–16, 18–24 | Need cloud / VR / datasets / HF | **N/A** or manual integration outside package |
| 7 | `raid_task_id` / `payment_id` in session | Real DATA_NODE upload + `metadata.json` / API — **not covered by rospy_x402 unit tests** |
| 17 | peaq claim in SessionRecord | Same: robot writes claim path (teleop_fetch); cloud record — out of repo |

---

## rospy_x402 automated tests

Run **2026-03-31**:

```bash
cd /home/ubuntu/ros_ws && source devel/setup.bash
catkin build rospy_x402 --no-status --catkin-make-args run_tests
```

Result: **34 tests OK** (`test_escalation_service`, `test_raid_*`, `test_dashboard_events_log`, etc.).

---

## Environment logs

`~/.kyr/dashboard_events.jsonl` may contain: `help_request_start`, `help_raid_post_ok`, `grant_poll_ok` / `grant_inline_ok`, `grant_mock_fallback`, `session_close` (from KYR) — **no** `TELEOP_TAKEOVER`.

---

## Notes for PM / Backend

- Sprint **Receipt emission** (table + HMAC + GET) is a separate service; rospy_x402 provides **x402 payment flow** and **RAID linkage**, not receipt CRUD.
- For item 7 and test 7, align **`task_id` from RAID** vs **`raid_task_id` / `payment_id`** in DATA_NODE contract (see teleop_fetch docs and Backend).
