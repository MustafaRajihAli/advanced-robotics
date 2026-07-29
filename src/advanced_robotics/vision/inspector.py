"""Inspector interface + a model-free stand-in.

`DefectDetector` is the production path, but it needs a trained ONNX model
(Phase 3). The orchestrator only depends on the `Inspector` protocol below,
which `DefectDetector.detect` already satisfies, so swapping the trained model
in is a one-line change at the call site.

`IntensityThresholdInspector` is a deliberately crude dark-region detector:
enough to close the Phase 5 loop end to end in simulation, not a substitute
for a trained defect model on a real inspection line.
"""
from __future__ import annotations

from typing import Protocol

import numpy as np

from advanced_robotics.core.types import DefectReport
from advanced_robotics.vision.camera_fusion import CameraFrame


class Inspector(Protocol):
    def detect(self, frame: CameraFrame) -> DefectReport: ...


class IntensityThresholdInspector:
    def __init__(self, dark_threshold: int = 60, min_area_px: int = 16) -> None:
        self.dark_threshold = dark_threshold
        self.min_area_px = min_area_px

    def detect(self, frame: CameraFrame) -> DefectReport:
        gray = frame.image.mean(axis=2) if frame.image.ndim == 3 else frame.image
        mask = gray < self.dark_threshold
        area = int(mask.sum())

        if area < self.min_area_px:
            return DefectReport(
                frame_id=_frame_id(frame),
                camera_id=frame.camera_id,
                defect_found=False,
                confidence=0.0,
            )

        rows, cols = np.nonzero(mask)
        bbox = (int(cols.min()), int(rows.min()), int(cols.max()) + 1, int(rows.max()) + 1)
        # Confidence stands in for a model score: how far the darkest pixel sits
        # below threshold, clamped to [0, 1].
        depth = (self.dark_threshold - float(gray[mask].min())) / max(self.dark_threshold, 1)
        return DefectReport(
            frame_id=_frame_id(frame),
            camera_id=frame.camera_id,
            defect_found=True,
            confidence=min(1.0, max(0.0, depth)),
            bbox_xyxy=bbox,
        )


def _frame_id(frame: CameraFrame) -> str:
    return f"{frame.camera_id}-{frame.timestamp_s:.3f}"
