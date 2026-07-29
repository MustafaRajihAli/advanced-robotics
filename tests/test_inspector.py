import numpy as np

from advanced_robotics.vision.camera_fusion import CameraFrame
from advanced_robotics.vision.inspector import IntensityThresholdInspector


def _frame(image: np.ndarray) -> CameraFrame:
    return CameraFrame(camera_id="cam0", image=image, timestamp_s=1.5)


def test_clean_frame_reports_no_defect():
    inspector = IntensityThresholdInspector()

    report = inspector.detect(_frame(np.full((32, 32, 3), 200, dtype=np.uint8)))

    assert report.defect_found is False
    assert report.confidence == 0.0
    assert report.bbox_xyxy is None


def test_dark_patch_is_bounded_and_scored():
    image = np.full((32, 32, 3), 200, dtype=np.uint8)
    image[8:16, 4:12] = 0
    inspector = IntensityThresholdInspector()

    report = inspector.detect(_frame(image))

    assert report.defect_found is True
    assert report.bbox_xyxy == (4, 8, 12, 16)
    assert report.confidence == 1.0
    assert report.frame_id == "cam0-1.500"


def test_patch_below_min_area_is_ignored_as_noise():
    image = np.full((32, 32, 3), 200, dtype=np.uint8)
    image[0:2, 0:2] = 0  # 4 px, under the 16 px floor
    inspector = IntensityThresholdInspector()

    assert inspector.detect(_frame(image)).defect_found is False
