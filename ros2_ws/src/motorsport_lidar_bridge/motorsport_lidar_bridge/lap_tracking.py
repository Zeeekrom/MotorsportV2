"""Closed-circle progress and truth metrics. / 闭合圆形赛道进度及真值指标。"""

from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np


@dataclass(frozen=True)
class CircleTruth:
    """Ground-truth metrics at one vehicle pose. / 单个车辆位姿的真值指标。"""

    progress_m: float
    progress_percent: float
    completed_laps: int
    cross_track_error_m: float
    heading_error_degrees: float
    distance_to_start_m: float


def wrap_degrees(angle_degrees: float) -> float:
    """Wrap an angle to [-180, 180). / 把角度归一化到[-180, 180)。"""
    return (float(angle_degrees) + 180.0) % 360.0 - 180.0


class CircularLapTracker:
    """Unwrap circular progress across the start line. / 跨越起终点连续展开圆周进度。"""

    def __init__(self, radius_m: float) -> None:
        if radius_m <= 0.0:
            raise ValueError("radius_m must be positive. / radius_m必须为正数。")
        self.radius_m = float(radius_m)
        self.circumference_m = 2.0 * math.pi * self.radius_m
        self.last_angle_rad: float | None = None
        self.unwrapped_angle_rad = 0.0

    def update(
        self,
        position_xy: np.ndarray,
        heading_degrees: float,
    ) -> CircleTruth:
        """Update lap progress from local position and heading. / 根据局部位置和航向更新圈进度。"""
        position = np.asarray(position_xy, dtype=np.float64).reshape(-1)[:2]
        x_position = float(position[0])
        y_position = float(position[1])

        # Course centre is (0, radius); start is (0, 0), heading +X.
        # 圆心为(0, radius)，起点为(0, 0)，起始航向为+X。
        angle = math.atan2(
            x_position,
            self.radius_m - y_position,
        )
        if self.last_angle_rad is None:
            self.last_angle_rad = angle
        else:
            delta = (angle - self.last_angle_rad + math.pi) % (
                2.0 * math.pi
            ) - math.pi
            self.unwrapped_angle_rad += delta
            self.last_angle_rad = angle

        progress_m = max(0.0, self.unwrapped_angle_rad * self.radius_m)
        progress_percent = 100.0 * progress_m / self.circumference_m
        completed_laps = int(progress_m // self.circumference_m)
        radial_distance = math.hypot(
            x_position,
            y_position - self.radius_m,
        )
        cross_track_error = abs(radial_distance - self.radius_m)
        reference_heading = math.degrees(angle)
        heading_error = wrap_degrees(heading_degrees - reference_heading)
        distance_to_start = math.hypot(x_position, y_position)
        return CircleTruth(
            progress_m=progress_m,
            progress_percent=progress_percent,
            completed_laps=completed_laps,
            cross_track_error_m=cross_track_error,
            heading_error_degrees=heading_error,
            distance_to_start_m=distance_to_start,
        )
