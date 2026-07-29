# Architecture

## Overview

```
                    +-------------------+
                    |   Plant systems   |
                    |  ERP / MES / SCADA|
                    +---------+---------+
                              | integrations/
                              v
+------------------+   +-----------+   +------------------+
| ros2_ws (ROS 2)  |<->|  api/     |<->|  safety/         |
| bridge_node       |   | FastAPI   |   | SafetyMonitor    |
+---------+---------+   +-----+-----+   | + EstopSource    |
          |                    |         +--------+---------+
          v                    v                  |
   +-------------+     +--------------+            | check() gates every
   |   amr/      |     |   arm/       |            | actuator command
   | SLAM, Nav,  |     | Kinematics,  |<-----------+
   | Fleet coord |     | ForceTorque, |
   +-------------+     | MotionPlan   |
          ^             +-------------+
          |
   +-------------+
   |  vision/    |
   | CameraFusion|
   | DefectDetect|
   +-------------+
          ^
          |
   +----------------+
   | digital_twin/  |
   | SimulatorBackend|  <- same interfaces used for sim and real hardware
   +----------------+
```

## Design principles

1. **Safety is not optional.** Every actuator-facing loop (velocity commands
   in `amr/navigation.py`, trajectory execution in `arm/motion_planner.py`)
   must call `SafetyMonitor.check()` before issuing a command. A
   `SafetyFaultError` means stop, full stop — no partial actuation.

2. **Hardware/sim parity.** `digital_twin/simulator.py` defines
   `SimulatorBackend`, the same shape of interface real ROS 2 drivers expose.
   AMR/arm/vision logic is written against these interfaces, not against
   ROS 2 or a specific sim engine directly, so the same code path runs in
   the digital twin and on real hardware.

3. **ROS 2 stays at the edge.** Only `ros2_ws/` imports `rclpy`. Everything
   under `src/advanced_robotics/` is plain Python so it can be unit tested
   without a ROS 2 environment installed.

4. **Plant integration is pluggable, not hardcoded.** `erp_client.py`,
   `mes_client.py`, and `scada_client.py` all implement `PlantSystemClient`
   so the task-processing loop doesn't care which system a task came from.

## Data flow: a manufacturing task end to end

1. MES posts a job -> `integrations/mes_client.py` polls it into a `PlantTask`.
2. Task handed to `amr/fleet_coordinator.py` to route a robot to the part,
   or directly to `arm/motion_planner.py` if it's a stationary arm task.
3. Before every velocity/trajectory command: `safety/safety_monitor.py`
   check gate.
4. `vision/defect_detector.py` inspects the result; `DefectReport` sent back
   through `integrations/mes_client.py.report_status()`.
5. Everything above runs identically whether the "robot" is
   `digital_twin/simulator.py` or the ROS 2 bridge talking to real hardware.

## What's stubbed vs. real

- Real: type contracts, config schema, safety gating logic, kinematics math,
  fleet allocation logic, API surface, ROS 2 package structure.
- Stubbed (see `WORK_PLAN.md` phases): actual SLAM/Nav2 wiring, analytic IK
  for a specific arm model, trained defect-detection model, Gazebo/Isaac Sim
  backend, ERP/MES/SCADA endpoint schemas (placeholder REST shapes above —
  confirm against the real systems before Phase 5).
