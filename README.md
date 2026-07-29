# Advanced Robotics

Control software for the Novus AI Dynamics "Advanced Robotics" service line:
https://novusaidynamics.com/services/advanced-robotics.html

Four subsystems on one platform:
- **AMR** — autonomous mobile robot fleet navigation (SLAM, obstacle avoidance, fleet coordination)
- **Arm** — collaborative robot arm control (kinematics, force-torque compliance, motion planning)
- **Vision** — AI inspection (multi-camera/LiDAR fusion, edge defect detection)
- **Safety** — e-stop integration and safety monitoring (ISO 10218-oriented)

Plus an **integrations** layer for ERP/MES/SCADA connectivity and a **digital twin**
simulation harness for pre-deployment testing.

See [`WORK_PLAN.md`](./WORK_PLAN.md) for the phased build plan and the
website-claim-to-engineering-scope mapping.

## Layout

```
src/advanced_robotics/   Python application code (core, amr, arm, vision, safety, integrations, digital_twin, api)
ros2_ws/                 ROS 2 workspace: bridge node between ROS 2 topics and the Python app layer
simulation/              Digital twin sim assets
config/                  Runtime configuration
tests/                   Unit tests
docs/                    Architecture notes
scripts/                 Dev/setup scripts
```

## Status

Scaffold stage (Phase 0). Modules contain real interfaces and typed contracts
but not production control loops yet — see `WORK_PLAN.md` for what's next.

## Quickstart

```bash
python -m venv .venv
source .venv/bin/activate   # or .venv\Scripts\activate on Windows
pip install -r requirements.txt
cp .env.example .env
pytest
```
