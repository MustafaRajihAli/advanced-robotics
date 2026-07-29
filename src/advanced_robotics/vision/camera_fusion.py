"""Multi-camera + LiDAR frame alignment.

Timestamp-aligns frames from multiple cameras and a LiDAR sweep into a
single synchronized FusedFrame so downstream inspection/navigation code
doesn't have to reason about per-sensor timing.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class CameraFrame:
    camera_id: str
    image: np.ndarray
    timestamp_s: float


@dataclass(frozen=True)
class LidarScan:
    lidar_id: str
    points_xyz: np.ndarray  # (N, 3)
    timestamp_s: float


@dataclass(frozen=True)
class FusedFrame:
    timestamp_s: float
    camera_frames: dict[str, CameraFrame]
    lidar_scan: LidarScan | None


class CameraFusion:
    def __init__(self, max_sync_skew_s: float = 0.05) -> None:
        self.max_sync_skew_s = max_sync_skew_s

    def fuse(
        self, camera_frames: list[CameraFrame], lidar_scan: LidarScan | None = None
    ) -> FusedFrame:
        if not camera_frames:
            raise ValueError("need at least one camera frame to fuse")

        reference_ts = camera_frames[0].timestamp_s
        for frame in camera_frames:
            if abs(frame.timestamp_s - reference_ts) > self.max_sync_skew_s:
                raise ValueError(
                    f"camera {frame.camera_id} frame out of sync by "
                    f"{abs(frame.timestamp_s - reference_ts):.3f}s"
                )

        if lidar_scan is not None and abs(lidar_scan.timestamp_s - reference_ts) > self.max_sync_skew_s:
            lidar_scan = None  # drop stale lidar rather than fail the whole frame

        return FusedFrame(
            timestamp_s=reference_ts,
            camera_frames={f.camera_id: f for f in camera_frames},
            lidar_scan=lidar_scan,
        )
