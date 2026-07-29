import numpy as np
import pytest

from advanced_robotics.vision.camera_fusion import CameraFrame, CameraFusion


def _frame(camera_id: str, ts: float) -> CameraFrame:
    return CameraFrame(camera_id=camera_id, image=np.zeros((4, 4, 3), dtype=np.uint8), timestamp_s=ts)


def test_fuses_in_sync_frames():
    fusion = CameraFusion(max_sync_skew_s=0.05)
    result = fusion.fuse([_frame("cam0", 1.0), _frame("cam1", 1.01)])
    assert set(result.camera_frames.keys()) == {"cam0", "cam1"}


def test_rejects_out_of_sync_frames():
    fusion = CameraFusion(max_sync_skew_s=0.01)
    with pytest.raises(ValueError):
        fusion.fuse([_frame("cam0", 1.0), _frame("cam1", 1.5)])
