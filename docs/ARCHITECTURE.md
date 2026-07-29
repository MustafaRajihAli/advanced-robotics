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
          |            +-------v--------+          |
          |            | orchestration/ |          | check() gates every
          |            | TaskRouter     |          | actuator command
          |            | TaskExecutor   |----------+
          |            +---+--------+---+
          v                |        |
   +-------------+         |        |   +--------------+
   |   amr/      |<--------+        +-->|   arm/       |
   | SLAM, Nav,  |                      | Kinematics,  |
   | Fleet coord |                      | ForceTorque, |
   +-------------+                      | MotionPlan   |
          ^                             +--------------+
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
   `mes_client.py`, `scada_client.py`, and `in_memory_plant.py` all implement
   `PlantSystemClient` so the task-processing loop doesn't care which system a
   task came from — the end-to-end tests run the production executor against
   the in-memory client.

5. **Failures are reported, never swallowed.** Every task ends in a
   `TaskOutcome` whose status (`completed`, `halted`, `blocked`, `timeout`,
   `rejected`, `failed`) is pushed back to the plant system. A task that faults
   still releases its robot — `TaskExecutor.execute` does that in a `finally`,
   because a robot stuck in `busy` starves the fleet.

6. **Payloads are validated at the edge.** `orchestration/task_router.py` is
   the only place that reads plant payload shapes. An unparseable goal is
   rejected upstream rather than becoming a motion command with a guessed
   target.

## Data flow: a manufacturing task end to end

Implemented in `orchestration/task_executor.py`; exercised by
`tests/test_task_executor.py` and `scripts/run_demo.py`.

1. MES posts a job -> `integrations/*` polls it into a `PlantTask`
   (`TaskExecutor.run_once` claims every pending task).
2. `orchestration/task_router.py` validates the payload and produces a typed
   `RobotJob`. Unroutable tasks are reported back as `rejected` and never reach
   an actuator.
3. Mobile jobs go to `amr/fleet_coordinator.py`, which assigns the nearest idle
   robot — after the executor refreshes the coordinator's poses from the world,
   since allocation is distance-based. `pick_place` jobs bypass allocation and
   go straight to `arm/motion_planner.py`.
4. Per tick: pump the heartbeat, `safety/safety_monitor.py` check gate, compute
   a velocity command from `amr/navigation.py`, step the world. A
   `SafetyFaultError` zeroes velocity and ends the task as `halted`.
5. On arrival, an `inspect` job pulls frames via the simulator's camera
   interface and runs `vision/inspector.py`; the `DefectReport`s ride back
   inside the status report.
6. The robot is released and `report_status()` sends the outcome upstream.
7. Everything above runs identically whether the world is
   `digital_twin/kinematic_backend.py`, Gazebo, Isaac Sim, or the ROS 2 bridge
   talking to real hardware — the executor only knows `SimulatorBackend`.

## What's stubbed vs. real

- Real: type contracts, config schema, safety gating logic, kinematics math,
  fleet allocation, the end-to-end task path (routing, allocation, safety-gated
  motion, inspection, status reporting), the kinematic simulator backend, the
  API surface, and the ROS 2 package structure.
- Stubbed (see `WORK_PLAN.md` phases): actual SLAM/Nav2 wiring, analytic IK for
  a specific arm model, the trained defect-detection model
  (`IntensityThresholdInspector` is a deliberate model-free stand-in), the
  Gazebo/Isaac Sim backends and RL harness, arm-side simulation (trajectories
  are planned and safety-gated but not physically executed), and the
  ERP/MES/SCADA endpoint schemas — the placeholder REST shapes above still need
  confirming against the real systems before this talks to a live plant.
