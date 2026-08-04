"""Pure-pursuit controller for the planned cone centre path. / 锥桶中心路径纯追踪控制器。"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class PathControl:
    """Controller output independent of CARLA and ROS. / 与CARLA和ROS无关的控制器输出。"""

    valid: bool
    target_speed_mps: float
    steering_normalized: float
    curvature_per_m: float
    lookahead_distance_m: float
    cross_track_error_m: float
    target_point: np.ndarray


def stopped_control() -> PathControl:
    """Return a safe no-path command. / 返回安全的无路径停车命令。"""
    return PathControl(
        valid=False,
        target_speed_mps=0.0,
        steering_normalized=0.0,
        curvature_per_m=0.0,
        lookahead_distance_m=0.0,
        cross_track_error_m=0.0,
        target_point=np.zeros(2, dtype=np.float32),
    )


def pure_pursuit_control(
    path_points: np.ndarray,
    vehicle_position: np.ndarray,
    forward_direction: np.ndarray,
    *,
    requested_speed_mps: float = 2.0,
    lookahead_distance_m: float = 5.0,
    wheelbase_m: float = 2.875,
    maximum_steering_angle_degrees: float = 35.0,
    minimum_ahead_points: int = 2,
) -> PathControl:
    """Calculate speed and steering from an ordered local path. / 根据有序局部路径计算速度和转向。"""
    path = np.asarray(path_points, dtype=np.float32)
    if path.size == 0 or path.ndim != 2 or path.shape[1] < 2:
        return stopped_control()
    position = np.asarray(vehicle_position, dtype=np.float32).reshape(-1)[:2]
    forward = np.asarray(forward_direction, dtype=np.float32).reshape(-1)[:2]
    forward_norm = float(np.linalg.norm(forward))
    if forward_norm < 1e-6:
        raise ValueError(
            "forward_direction must be non-zero / forward_direction不能是零向量"
        )
    forward /= forward_norm
    left_axis = np.asarray([-forward[1], forward[0]], dtype=np.float32)

    # Ignore points behind the front axle. / 忽略前轴后方的路径点。
    relative = path[:, :2] - position
    longitudinal = relative @ forward
    ahead = longitudinal > 0.20
    if np.count_nonzero(ahead) < max(1, int(minimum_ahead_points)):
        return stopped_control()
    path_ahead = path[ahead, :2]
    relative_ahead = path_ahead - position
    distances = np.linalg.norm(relative_ahead, axis=1)

    # Select the first point at or beyond the requested look-ahead.
    # 选择第一个达到或超过目标前视距离的点。
    far_enough = np.flatnonzero(distances >= lookahead_distance_m)
    target_index = int(far_enough[0]) if len(far_enough) else len(path_ahead) - 1
    target = path_ahead[target_index]
    target_relative = target - position
    target_distance = max(float(np.linalg.norm(target_relative)), 1e-3)
    lateral_target = float(target_relative @ left_axis)

    # Standard pure-pursuit curvature and Ackermann steering angle.
    # 使用标准纯追踪曲率和阿克曼转向角。
    curvature = 2.0 * lateral_target / (target_distance**2)
    steering_angle = float(np.arctan(wheelbase_m * curvature))
    maximum_angle = float(np.radians(maximum_steering_angle_degrees))
    steering_normalized = float(
        np.clip(steering_angle / maximum_angle, -1.0, 1.0)
    )

    # Reduce speed on sharper curves while preserving a useful crawl speed.
    # 弯道越急速度越低，同时保留可用的最低慢行速度。
    requested_speed = max(float(requested_speed_mps), 0.0)
    target_speed = requested_speed / (1.0 + 4.0 * abs(curvature))
    target_speed = min(requested_speed, max(0.8, target_speed))

    nearest_index = int(np.argmin(distances))
    cross_track_error = float(
        relative_ahead[nearest_index] @ left_axis
    )
    return PathControl(
        valid=True,
        target_speed_mps=target_speed,
        steering_normalized=steering_normalized,
        curvature_per_m=curvature,
        lookahead_distance_m=target_distance,
        cross_track_error_m=cross_track_error,
        target_point=target.astype(np.float32, copy=True),
    )
