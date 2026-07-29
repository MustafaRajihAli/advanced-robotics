import pytest
from fastapi.testclient import TestClient

from advanced_robotics.api.app import create_app
from advanced_robotics.orchestration.bootstrap import build_simulation_stack


@pytest.fixture
def stack():
    return build_simulation_stack(defect_camera_ids=frozenset({"cam0"}))


@pytest.fixture
def client(stack):
    return TestClient(create_app(stack))


def test_health(client):
    assert client.get("/health").json() == {"status": "ok"}


def test_robots_report_live_sim_poses(client, stack):
    robots = client.get("/robots").json()

    assert len(robots) == stack.config.amr.fleet_size
    assert robots[0]["pose"] == {"x": 0.0, "y": 0.0, "yaw_rad": 0.0}
    assert robots[0]["busy"] is False


def test_submitted_task_runs_end_to_end_through_the_api(client):
    response = client.post(
        "/tasks", json={"task_id": "job-1", "task_type": "transport", "payload": {"x": 3.0, "y": 0.0}}
    )
    assert response.status_code == 202
    assert client.get("/tasks").json()["queued"] == ["job-1"]

    outcomes = client.post("/tasks/run").json()

    assert [o["status"] for o in outcomes] == ["completed"]
    assert outcomes[0]["robot_id"] == "amr0"
    assert client.get("/tasks").json() == {"queued": [], "executed": ["job-1"]}
    assert client.get("/outcomes").json()[0]["task_id"] == "job-1"
    assert client.get("/robots").json()[0]["pose"]["x"] > 2.0


def test_estop_shows_in_safety_status_and_blocks_reset_until_released(client, stack):
    stack.estop.trigger()
    client.post("/tasks", json={"task_id": "job-2", "task_type": "transport", "payload": {"x": 3.0, "y": 0.0}})

    assert client.post("/tasks/run").json()[0]["status"] == "halted"

    status = client.get("/safety").json()
    assert status["estop_triggered"] is True
    assert status["mode"] == "estopped"

    # Reset is refused while the e-stop is still engaged.
    assert client.post("/safety/reset").status_code == 409

    stack.estop.reset()
    reset = client.post("/safety/reset").json()
    assert reset["mode"] == "idle"
    assert reset["estop_triggered"] is False


def test_default_app_builds_without_an_injected_stack():
    app = create_app()
    with TestClient(app) as default_client:
        assert default_client.get("/health").status_code == 200
        assert len(default_client.get("/robots").json()) > 0
