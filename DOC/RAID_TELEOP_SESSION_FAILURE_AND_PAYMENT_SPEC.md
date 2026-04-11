# RAID App — teleop session failure visibility and operator payment (robot-side)

**Audience:** RAID App / product / support.  
**Robot:** `teleop_fetch`, `br-kyr` (`kyr_proxy`), `rospy_x402` (`/x402/complete_teleop_payment`).  
**Related:** [`RAID_APP_TELEOP_HELP_FULL_CYCLE_X402_SPEC.md`](RAID_APP_TELEOP_HELP_FULL_CYCLE_X402_SPEC.md) (happy path).

## 1. Scope

Document how the robot behaves when the **teleoperator is lost** (network, app kill, ping policy in VR, battery) and how **SOL payment** is computed in the **interim** policy. RAID HTTP contracts for `teleop/help` and SessionGrant **do not change** for this feature set.

## 2. Session end triggers (reminder)

| Trigger | Robot component |
|---------|-----------------|
| ROS `/teleop_fetch/end_session` | `teleop_fetch` → `/kyr/close_session` → optional `/x402/complete_teleop_payment` |
| Second L_Y (if enabled) | same chain |
| Watchdog / operator loss | `teleop_fetch` → same chain with **abnormal** `closure_reason` |

Until `/kyr/close_session` runs, KYR session stays **ACTIVE**; RAID must not assume billing from “operator left UI” alone.

## 3. `closure_reason` strings (informative)

Robots emit free-text `reason` into KYR `close_session`; it is copied into **SignedReceipt** as `closure_reason`. Examples:

- **Normal / full pay (multiplier 1.0):** `operator_finished`, `end_session_service`, `ly_second_press` (exact set configurable on robot).
- **Abnormal / interim fractional pay:** `robot_watchdog_timeout`, `operator_disconnect_ping`, `operator_disconnect_network`, `operator_disconnect_app_exit`, `operator_disconnect_power`, `operator_disconnect_unknown`, and prefix matches configured on `rospy_x402`.

## 4. Interim payment policy (robot)

After `close_session`, `teleop_fetch` calls `/x402/complete_teleop_payment` with the receipt payload.

**Interim rule:** if `closure_reason` is classified as **abnormal**, the robot applies **`teleop_operator_abnormal_payment_fraction`** (default **0.5**) to the **amount** that would otherwise be computed from receipt / flat / per-second rules (see `teleop_operator_payment.py`).

**Observability:**

- ROS logs on `x402_ex_server` / payment module.
- Optional append to `~/.kyr/dashboard_events.jsonl` with kind `teleop_operator_payment` (metadata: amount, signature or skip reason, `closure_reason`) for KYR Black Box UI.

## 5. Changes required on your side (RAID)

| Item | Required? |
|------|-----------|
| Change `POST …/teleop/help` or SessionGrant schema | **No** |
| Show “session ended by watchdog” in operator UI | **Optional** — would need robot→RAID push or operator polling (not in mandatory contract today) |
| Ledger adjustment for 50% interim rule | **Optional** — robot pays less on-chain for abnormal reasons; RAID may display help id + amount if notified later |

**Mandatory RAID backend change:** **none** for this iteration.

## 6. References

- VR contract (liveness / lifecycle): [`../../br-vr-dev-sinc/DOC/VR_APP_TELEOP_ROS_CONTRACT.md`](../../br-vr-dev-sinc/DOC/VR_APP_TELEOP_ROS_CONTRACT.md)
- Full cycle: [`RAID_APP_TELEOP_HELP_FULL_CYCLE_X402_SPEC.md`](RAID_APP_TELEOP_HELP_FULL_CYCLE_X402_SPEC.md)
- Grant polling: [`ROBOT_TELEOP_KYR_RAID_GRANT.md`](ROBOT_TELEOP_KYR_RAID_GRANT.md)
