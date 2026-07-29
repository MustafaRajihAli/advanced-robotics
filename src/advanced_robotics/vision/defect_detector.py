"""Edge-inference defect detection.

Wraps an ONNX Runtime session behind a typed interface. Model path/backend
are config-driven so the same code runs whatever classifier is trained for
a given inspection line.

Postprocessing assumes a YOLO-family output layout (the dominant approach
for real-time industrial edge inspection: YOLOv8-class models hit >120 FPS
on Jetson Orin-class hardware while maintaining high mAP on defect
datasets like NEU-DET/MVTec AD) rather than a single global confidence
score, since real inspection scenes usually contain zero, one, or several
distinct defects that each need their own bounding box and confidence.
"""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import numpy as np

from advanced_robotics.core.types import DefectReport
from advanced_robotics.vision.camera_fusion import CameraFrame


@dataclass(frozen=True)
class Detection:
    bbox_xyxy: tuple[float, float, float, float]
    confidence: float
    class_id: int


class DefectDetector:
    def __init__(
        self,
        model_path: str | Path,
        confidence_threshold: float = 0.85,
        iou_threshold: float = 0.45,
        input_size: int = 640,
    ) -> None:
        self.model_path = Path(model_path)
        self.confidence_threshold = confidence_threshold
        self.iou_threshold = iou_threshold
        self.input_size = input_size
        self._session = None  # lazy-loaded; keeps import-time cost low for tests

    def _ensure_session(self):
        if self._session is None:
            import onnxruntime as ort

            self._session = ort.InferenceSession(str(self.model_path))
        return self._session

    def detect(self, frame: CameraFrame) -> DefectReport:
        session = self._ensure_session()
        input_name = session.get_inputs()[0].name
        blob = _preprocess(frame.image, self.input_size)
        raw_output = session.run(None, {input_name: blob})[0]

        detections = self._postprocess(raw_output)
        best = max(detections, key=lambda d: d.confidence, default=None)

        return DefectReport(
            frame_id=f"{frame.camera_id}-{frame.timestamp_s}",
            camera_id=frame.camera_id,
            defect_found=best is not None,
            confidence=best.confidence if best else 0.0,
            bbox_xyxy=tuple(int(v) for v in best.bbox_xyxy) if best else None,
        )

    def _postprocess(self, raw_output: np.ndarray) -> list[Detection]:
        """Parse a YOLOv8-style (1, 4+num_classes, num_boxes) output into
        Detections, filtered by confidence and de-duplicated with NMS."""
        candidates = parse_yolo_output(raw_output, self.confidence_threshold)
        return non_max_suppression(candidates, self.iou_threshold)


def parse_yolo_output(raw_output: np.ndarray, confidence_threshold: float) -> list[Detection]:
    """raw_output shape: (1, 4 + num_classes, num_boxes), boxes in cx,cy,w,h."""
    predictions = raw_output[0].T  # -> (num_boxes, 4 + num_classes)
    boxes_cxcywh = predictions[:, :4]
    class_scores = predictions[:, 4:]

    class_ids = np.argmax(class_scores, axis=1)
    confidences = class_scores[np.arange(len(class_scores)), class_ids]

    keep = confidences >= confidence_threshold
    boxes_cxcywh, class_ids, confidences = boxes_cxcywh[keep], class_ids[keep], confidences[keep]

    detections = []
    for (cx, cy, w, h), class_id, confidence in zip(boxes_cxcywh, class_ids, confidences):
        x1, y1, x2, y2 = cx - w / 2, cy - h / 2, cx + w / 2, cy + h / 2
        detections.append(Detection((x1, y1, x2, y2), float(confidence), int(class_id)))
    return detections


def non_max_suppression(detections: list[Detection], iou_threshold: float) -> list[Detection]:
    if not detections:
        return []

    ordered = sorted(detections, key=lambda d: d.confidence, reverse=True)
    kept: list[Detection] = []

    while ordered:
        current = ordered.pop(0)
        kept.append(current)
        ordered = [d for d in ordered if _iou(current.bbox_xyxy, d.bbox_xyxy) < iou_threshold]

    return kept


def _iou(box_a: tuple[float, float, float, float], box_b: tuple[float, float, float, float]) -> float:
    ax1, ay1, ax2, ay2 = box_a
    bx1, by1, bx2, by2 = box_b

    inter_x1, inter_y1 = max(ax1, bx1), max(ay1, by1)
    inter_x2, inter_y2 = min(ax2, bx2), min(ay2, by2)
    inter_area = max(0.0, inter_x2 - inter_x1) * max(0.0, inter_y2 - inter_y1)

    area_a = max(0.0, ax2 - ax1) * max(0.0, ay2 - ay1)
    area_b = max(0.0, bx2 - bx1) * max(0.0, by2 - by1)
    union = area_a + area_b - inter_area

    return inter_area / union if union > 0 else 0.0


def _preprocess(image: np.ndarray, input_size: int) -> np.ndarray:
    """Resize (nearest, no external dep), normalize HWC uint8 to NCHW float32 in [0, 1]."""
    resized = _resize_nearest(image, input_size, input_size)
    normalized = resized.astype(np.float32) / 255.0
    chw = np.transpose(normalized, (2, 0, 1))
    return np.expand_dims(chw, axis=0)


def _resize_nearest(image: np.ndarray, width: int, height: int) -> np.ndarray:
    src_h, src_w = image.shape[:2]
    row_idx = (np.arange(height) * src_h / height).astype(int)
    col_idx = (np.arange(width) * src_w / width).astype(int)
    return image[row_idx][:, col_idx]
