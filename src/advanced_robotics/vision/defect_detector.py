"""Edge-inference defect detection.

Wraps an ONNX Runtime session behind a typed interface. Model path/backend
are config-driven so the same code runs whatever classifier is trained for
a given inspection line.
"""
from __future__ import annotations

from pathlib import Path

import numpy as np

from advanced_robotics.core.types import DefectReport
from advanced_robotics.vision.camera_fusion import CameraFrame


class DefectDetector:
    def __init__(self, model_path: str | Path, confidence_threshold: float = 0.85) -> None:
        self.model_path = Path(model_path)
        self.confidence_threshold = confidence_threshold
        self._session = None  # lazy-loaded; keeps import-time cost low for tests

    def _ensure_session(self):
        if self._session is None:
            import onnxruntime as ort

            self._session = ort.InferenceSession(str(self.model_path))
        return self._session

    def detect(self, frame: CameraFrame) -> DefectReport:
        session = self._ensure_session()
        input_name = session.get_inputs()[0].name
        blob = _preprocess(frame.image)
        outputs = session.run(None, {input_name: blob})

        confidence = float(outputs[0].max())
        defect_found = confidence >= self.confidence_threshold

        return DefectReport(
            frame_id=f"{frame.camera_id}-{frame.timestamp_s}",
            camera_id=frame.camera_id,
            defect_found=defect_found,
            confidence=confidence,
        )


def _preprocess(image: np.ndarray) -> np.ndarray:
    """Normalize HWC uint8 image to NCHW float32 in [0, 1]."""
    normalized = image.astype(np.float32) / 255.0
    chw = np.transpose(normalized, (2, 0, 1))
    return np.expand_dims(chw, axis=0)
