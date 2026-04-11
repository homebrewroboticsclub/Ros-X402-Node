# DEV_AI — agent context (rospy_x402)

## Ecosystem entry point

**Launch, RAID parameters, and full-stack commands** — metapackage **`br_bringup`**:  
**[../br_bringup/DEV_AI.md](../br_bringup/DEV_AI.md)** and **[../br_bringup/README.md](../br_bringup/README.md)**.

## This repository

ROS package **`rospy_x402`**: configurable REST API, x402 (Solana) payment flows, RAID_APP enroll / `teleop/help` / operator sync, teleop payment completion services.

**Language (agents):** use **English** in this repository (code, comments, `DOC/`, operator-facing strings). In **chat**, answer in **Russian** when the human writes in Russian. Workspace rules: `ros_ws/.cursor/rules/project-context.mdc`. Public push checklist: [../br_bringup/DOC/PUBLIC_RELEASE_CHECKLIST.md](../br_bringup/DOC/PUBLIC_RELEASE_CHECKLIST.md).

## Related repositories (same overlay)

| Area | DEV_AI | DOC / README |
|------|--------|----------------|
| Ecosystem launch | [../br_bringup/DEV_AI.md](../br_bringup/DEV_AI.md) | [../br_bringup/README.md](../br_bringup/README.md) |
| KYR (policy, sessions) | [../br-kyr/DEV_AI.md](../br-kyr/DEV_AI.md) | [../br-kyr/DOC/README.md](../br-kyr/DOC/README.md) |
| VR teleop (`teleop_fetch` sources) | [../br-vr-dev-sinc/DEV_AI.md](../br-vr-dev-sinc/DEV_AI.md) | [../br-vr-dev-sinc/DOC/README.md](../br-vr-dev-sinc/DOC/README.md) |

Secrets template: [.env.example](.env.example) (do not commit real `.env`).

## Layout

- Python nodes and library under `src/rospy_x402/`.
- Endpoint JSON: `config/endpoints.example.json` (and overrides via `~config_path`).
- **Documentation index:** [DOC/README.md](DOC/README.md). Human-oriented setup and examples: [README.md](README.md).

## Responsibilities when changing code

1. **Documentation** — update the matching `DOC/*.md` files; new functional area → new `DOC/` file + line in [DOC/README.md](DOC/README.md). If RAID or launch defaults change, sync **[../br_bringup/launch/ecosystem.launch](../br_bringup/launch/ecosystem.launch)** and **[../br_bringup/README.md](../br_bringup/README.md)** when those values are owned by bringup.
2. **Tests** — new behaviour with tests; run the package test target if registered:  
   `cd /home/ubuntu/ros_ws && source devel/setup.bash && catkin_make run_tests --pkg rospy_x402` (or `catkin build --no-deps --pkg rospy_x402` when no tests).
3. **Commit** — clear English messages; no secrets in tree.

## Build and run (short)

```bash
cd /home/ubuntu/ros_ws/devel && source setup.bash
cd /home/ubuntu/ros_ws && catkin build rospy_x402
rosrun rospy_x402 x402_ex_server.py
```

Full stack: `roslaunch br_bringup ecosystem.launch` — see bringup docs.
