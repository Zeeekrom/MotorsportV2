"""Tests for ROS map conversion. / ROS 地图坐标转换测试。"""

import numpy as np

from motorsport_lidar_bridge.coordinates import world_to_ros_map


def test_initial_vehicle_translation_and_y_flip():
    """The origin moves to zero and Y changes sign. / 初始位置归零并翻转 Y 方向。"""
    world_to_local = np.eye(4, dtype=np.float32)
    world_to_local[0, 3] = -100.0
    world_to_local[1, 3] = -50.0
    points = np.asarray([[101.0, 52.0, 3.0]], dtype=np.float32)

    converted = world_to_ros_map(points, world_to_local)

    np.testing.assert_allclose(converted, [[1.0, -2.0, 3.0]])


def test_empty_point_cloud_stays_empty():
    """An empty scan stays valid. / 空点云仍然保持有效。"""
    converted = world_to_ros_map(
        np.empty((0, 3), dtype=np.float32),
        np.eye(4, dtype=np.float32),
    )
    assert converted.shape == (0, 3)
