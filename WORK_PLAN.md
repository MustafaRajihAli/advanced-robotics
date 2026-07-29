# Advanced Robotics — Work Plan

Source: https://novusaidynamics.com/services/advanced-robotics.html

This plan turns the marketing claims on the service page into a buildable
system. It covers four subsystems that share one platform: **AMR fleet
navigation**, **collaborative arm control**, **AI vision inspection**, and
the **safety/integration layer** that ties them to plant systems (ERP/MES/SCADA)
and to a digital twin for pre-deployment testing.

## Website claims -> engineering scope

| Claim on the page | What it actually requires |
|---|---|
| AMR fleet, 99.8% uptime | SLAM navigation, dynamic obstacle avoidance, fleet coordinator, health telemetry |
| Collaborative robot arms, force-torque sensing | Kinematics/motion planning, force-torque control loop, teach-pendant style programming interface |
| LiDAR + multi-camera fusion | Sensor drivers, point-cloud/image fusion pipeline feeding both AMR and vision modules |
| ROS 2 framework | `ros2_ws/` workspace with a bridge package connecting ROS 2 topics to the Python app layer |
| Reinforcement learning agents | Training harness in `digital_twin/`, sim-to-real hand-off contract |
| Edge AI inference | On-device model runner (ONNX/TensorRT), no hard cloud dependency |
| 100% inspection coverage | Vision defect-detection pipeline with camera fusion module |
| Digital twin simulation | Sim environment for testing AMR/arm/vision code before touching hardware |
| ISO 10218 safety monitoring + e-stop | Dedicated `safety/` module: watchdog, e-stop integration, safety-rated I/O |
| ERP/MES/SCADA integration | `integrations/` clients with a standardized internal API contract |
| Sub-millimeter positioning | Calibration + closed-loop feedback in arm control, not just open-loop kinematics |

## Phases

### Phase 0 — Foundations (this scaffold)
- Repo layout, `pyproject.toml`, `requirements.txt`, `docker-compose.yml`, `.env.example`
- Shared `core/` types and config loader used by every subsystem
- ROS 2 bridge package skeleton
- CI-friendly test scaffolding

### Phase 1 — AMR navigation
- `amr/slam.py`: SLAM front-end (wraps an existing SLAM stack, e.g. `slam_toolbox`, via ROS 2)
- `amr/navigation.py`: path planning + dynamic obstacle avoidance (Nav2-based)
- `amr/fleet_coordinator.py`: multi-robot task allocation and collision-free scheduling
- Success criteria: single simulated robot completes a mapped route in the digital twin with injected dynamic obstacles

### Phase 2 — Collaborative arm control
- `arm/kinematics.py`: forward/inverse kinematics for a configurable arm (DH parameters from config)
- `arm/force_torque.py`: force-torque sensor read loop + compliance control
- `arm/motion_planner.py`: trajectory generation with collision checking
- Success criteria: simulated arm executes a pick-and-place trajectory while respecting a force limit

### Phase 3 — AI vision inspection
- `vision/camera_fusion.py`: multi-camera + LiDAR frame alignment
- `vision/defect_detector.py`: edge-inference defect classifier (ONNX Runtime)
- Success criteria: defect detector runs on a recorded frame set and reports coverage %, precision/recall against labeled data

### Phase 4 — Safety layer
- `safety/estop.py`: hardware e-stop integration contract (GPIO/fieldbus abstraction)
- `safety/safety_monitor.py`: watchdog that halts AMR/arm motion on fault, logs to audit trail
- Success criteria: injected fault (simulated e-stop, lost heartbeat) halts all actuation within a bounded time

### Phase 5 — Plant integration ✅
- `integrations/erp_client.py`, `mes_client.py`, `scada_client.py`: thin clients behind one interface (`PlantSystemClient`)
- `integrations/in_memory_plant.py`: mock plant system (same protocol, no network) for tests and the demo
- `orchestration/task_router.py`: validates inbound payloads and maps plant task types onto typed `RobotJob`s; unroutable tasks are reported back as `rejected` rather than coerced into motion
- `orchestration/task_executor.py`: the end-to-end loop — claim task, allocate robot, drive it with `SafetyMonitor.check()` before every command, inspect, release the robot in a `finally`, report status upstream
- `orchestration/bootstrap.py`: single place that wires the object graph, shared by the API, the demo, and the tests
- `digital_twin/kinematic_backend.py`: runnable `SimulatorBackend` (differential-drive integration, sensed obstacles, synthetic camera/LiDAR) so the loop actually executes in CI — Gazebo and Isaac Sim remain the fidelity/training backends
- `api/`: internal REST API exposing live robot poses, safety state with an audited reset, task submission, and `POST /tasks/run`
- Success criteria **met**: `tests/test_task_executor.py` submits a task through the mock MES client, drives a robot to its goal in the digital twin, and asserts the status report came back — plus e-stop halt, blocked-path, rejection, and multi-robot cases. `scripts/run_demo.py` shows the same path end to end.

### Phase 6 — Digital twin & sim-to-real
- `digital_twin/simulator.py`: simulation harness (Gazebo/Isaac Sim adapter) driving AMR + arm + vision modules through the same interfaces used on real hardware
- RL training harness for AMR/arm policies, with a documented sim-to-real transfer checklist

## Upgrade log

Applied after researching current (2025) practice in each subsystem:

| Area | Change | Source |
|---|---|---|
| AMR obstacle handling | Replaced hard stop threshold with Nav2 costmap-style proportional slowdown (`inflation_radius`, `cost_scaling_factor`) + rotate-in-place recovery after N stuck ticks, to avoid the documented "freezing robot problem" | [Nav2 docs](https://docs.nav2.org/tutorials/docs/using_3laws_supervisor.html), [dynamic obstacle avoidance research](https://arxiv.org/pdf/2505.00237) |
| Arm IK | Added joint-limit clamping + randomized-restart retries (numerical approximation of TRAC-IK's local-minima escape) | [TRAC-IK paper](http://irl.cs.brown.edu/pubs/trac-ik.pdf), [MoveIt IK solver docs](https://moveit.picknik.ai/main/doc/how_to_guides/trac_ik/trac_ik_tutorial.html) |
| Vision defect detection | Switched from single max-confidence score to YOLO-style multi-box parsing + NMS, matching the dominant edge-inspection approach (YOLOv8-class models, >120 FPS on Jetson Orin-class hardware) | [Edge AI industrial inspection survey](https://www.mdpi.com/1999-4893/18/8/510) |
| Safety | Added ISO/TS 15066 speed-and-separation-monitoring (SSM) protective-distance formula, gating any shared human/robot workspace alongside the existing e-stop/heartbeat check | [ISO/TS 15066 SSM implementation](https://pmc.ncbi.nlm.nih.gov/articles/PMC5117641/), [ISO/TS 15066 explained](https://www.automate.org/robotics/tech-papers/iso-ts-15066-explained) |
| Digital twin | Added concrete `IsaacSimBackend` / `GazeboBackend` stubs reflecting the train-in-Isaac-Sim, validate-in-Gazebo, deploy-to-ROS2-hardware pipeline used in current sim-to-real research | [Sim-to-real transfer: Isaac Sim to Gazebo to ROS 2](https://arxiv.org/abs/2501.02902) |
| CI | Added GitHub Actions workflow running ruff + pytest on every push/PR | — |
| Orchestration (Phase 5) | Added `orchestration/` (router + executor + bootstrap) and a dependency-free `KinematicSimulator`, so the plant-task → safety-gated motion → status-report path executes in CI instead of existing only as interfaces. Faults are reported upstream as typed statuses (`halted`, `blocked`, `timeout`, `rejected`) and the allocated robot is always released | — |

## Non-goals for the initial build
- Surgical robotics and hazardous-environment (nuclear/EOD) applications are out of scope for the first version — they need certified hardware and domain-specific safety cases far beyond a code scaffold. Revisit once the core platform (Phases 0-4) is proven.

## Directory map

```
Advanced Robotics/
  WORK_PLAN.md            <- this file
  README.md
  pyproject.toml / requirements.txt
  docker-compose.yml
  .env.example / .gitignore
  config/default.yaml      shared runtime config
  src/advanced_robotics/
    core/                  shared types, config loader, errors
    amr/                   SLAM, navigation, fleet coordination
    arm/                   kinematics, force-torque, motion planning
    vision/                camera fusion, defect detection, inspector interface
    safety/                e-stop, safety monitor
    integrations/          ERP/MES/SCADA clients + in-memory mock plant
    orchestration/         task router, task executor, stack bootstrap
    digital_twin/          simulator backends (kinematic, Gazebo, Isaac Sim)
    api/                   internal API exposed to plant systems
  ros2_ws/src/novus_robotics_bridge/   ROS 2 <-> Python bridge node
  simulation/digital_twin/  sim assets/worlds
  tests/                   unit tests per module
  docs/ARCHITECTURE.md
  scripts/                 setup/dev scripts
```
