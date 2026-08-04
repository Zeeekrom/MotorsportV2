"""Coordinate conversion helpers. / 坐标转换工具。"""

from __future__ import annotations

import numpy as np


def world_to_ros_map(
    world_points: np.ndarray,
    world_to_local_matrix: np.ndarray,
) -> np.ndarray:
    """Center CARLA world points and convert handedness. / 居中 CARLA 世界点并转换坐标手性。"""
    if len(world_points) == 0:
        return np.empty((0, 3), dtype=np.float32)

    # Move the initial vehicle position to the local origin. / 把车辆初始位置移动到局部原点。
    homogeneous = np.column_stack(
        (
            np.asarray(world_points, dtype=np.float32),
            np.ones(len(world_points), dtype=np.float32),
        )
    )
    local_points = (
        np.asarray(world_to_local_matrix, dtype=np.float32)
        @ homogeneous.T
    ).T[:, :3]

    # CARLA uses +Y right; ROS uses +Y left. / CARLA 的 +Y 向右，ROS 的 +Y 向左。
    local_points[:, 1] *= -1.0
    return np.ascontiguousarray(local_points, dtype=np.float32)
