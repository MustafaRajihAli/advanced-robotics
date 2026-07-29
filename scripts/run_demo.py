"""Phase 5 end-to-end demo: mock MES -> robot -> status report.

Runs four tasks through the real orchestration path in the kinematic digital
twin -- a transport job, an inspection that finds a planted defect, an arm
pick-and-place, and a job that a triggered e-stop halts -- then prints what
was reported back to the plant system.

    python scripts/run_demo.py
"""
from __future__ import annotations

import asyncio
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from advanced_robotics.amr.navigation import Obstacle
from advanced_robotics.integrations.plant_client import PlantTask
from advanced_robotics.orchestration.bootstrap import build_simulation_stack


async def main() -> int:
    stack = build_simulation_stack(
        obstacles=[Obstacle(x=2.5, y=1.0, radius_m=0.4)],
        defect_camera_ids=frozenset({"cam0"}),
    )
    print(f"world '{stack.simulator.world}' with {len(stack.simulator.robot_ids)} AMRs\n")

    stack.plant_client.enqueue(PlantTask("MES-1001", "transport", {"x": 6.0, "y": 0.0}))
    stack.plant_client.enqueue(
        PlantTask("MES-1002", "inspection", {"x": 4.0, "y": 2.0, "camera_ids": ["cam0", "cam1"]})
    )
    stack.plant_client.enqueue(
        PlantTask("MES-1003", "pick_place", {"joint_goal_rad": [0.4, -0.2, 0.3, 0.0, 0.1, 0.0]})
    )

    for outcome in await stack.executor.run_once():
        _print_outcome(outcome)

    print("\n-- e-stop engaged mid-shift --")
    stack.estop.trigger()
    stack.plant_client.enqueue(PlantTask("MES-1004", "transport", {"x": 8.0, "y": 0.0}))
    for outcome in await stack.executor.run_once():
        _print_outcome(outcome)

    print(f"\nsafety mode: {stack.safety.mode.value}, audit events: {len(stack.safety.audit_log)}")
    print(f"status reports sent upstream: {len(stack.plant_client.reports)}")
    return 0


def _print_outcome(outcome) -> None:
    print(
        f"{outcome.task_id}  {outcome.status:<10} {outcome.robot_id or '-':<6} "
        f"{outcome.sim_time_s:6.2f}s sim  {outcome.detail}"
    )
    for defect in outcome.defects:
        mark = "DEFECT" if defect.defect_found else "clean "
        print(f"           {mark} {defect.camera_id} conf={defect.confidence:.2f} {defect.bbox_xyxy}")


if __name__ == "__main__":
    raise SystemExit(asyncio.run(main()))
