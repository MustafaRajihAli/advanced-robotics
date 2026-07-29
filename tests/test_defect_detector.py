import numpy as np
import pytest

from advanced_robotics.vision.defect_detector import (
    Detection,
    non_max_suppression,
    parse_yolo_output,
)


def _make_yolo_output(boxes_cxcywh: list[tuple[float, float, float, float]], confidences: list[float]) -> np.ndarray:
    """Build a (1, 4+num_classes, num_boxes) array with a single class."""
    num_boxes = len(boxes_cxcywh)
    output = np.zeros((1, 5, num_boxes), dtype=np.float32)
    for i, (cx, cy, w, h) in enumerate(boxes_cxcywh):
        output[0, 0, i] = cx
        output[0, 1, i] = cy
        output[0, 2, i] = w
        output[0, 3, i] = h
        output[0, 4, i] = confidences[i]
    return output


def test_parse_yolo_output_filters_low_confidence():
    raw = _make_yolo_output([(10, 10, 4, 4), (50, 50, 4, 4)], confidences=[0.9, 0.3])
    detections = parse_yolo_output(raw, confidence_threshold=0.5)
    assert len(detections) == 1
    assert detections[0].confidence == pytest.approx(0.9, abs=1e-4)


def test_non_max_suppression_removes_overlapping_boxes():
    boxes = [
        Detection(bbox_xyxy=(0, 0, 10, 10), confidence=0.95, class_id=0),
        Detection(bbox_xyxy=(1, 1, 11, 11), confidence=0.80, class_id=0),  # heavy overlap
        Detection(bbox_xyxy=(100, 100, 110, 110), confidence=0.85, class_id=0),  # separate
    ]
    kept = non_max_suppression(boxes, iou_threshold=0.5)
    assert len(kept) == 2
    assert kept[0].confidence == 0.95
    assert kept[1].confidence == 0.85


def test_non_max_suppression_empty_input():
    assert non_max_suppression([], iou_threshold=0.5) == []
