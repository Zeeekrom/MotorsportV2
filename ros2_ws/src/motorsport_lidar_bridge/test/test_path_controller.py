"""Tests for the pure-pursuit path controller. / 纯追踪路径控制器测试。"""

import numpy as np
import pytest

from motorsport_lidar_bridge.path_controller import pure_pursuit_control


def test_straight_path_commands_zero_steering():
    """A centred straight path should not steer. / 居中的直线路径不应转向。"""
    path = np.asarray([[4.0, 0.0], [8.0, 0.0], [12.0, 0.0]], dtype=np.float32)

    command = pure_pursuit_control(
        path,
        vehicle_position=np.asarray([0.0, 0.0]),
        forward_direction=np.asarray([1.0, 0.0]),
    )

    assert command.valid
    assert command.steering_normalized == pytest.approx(0.0, abs=1e-7)
    assert command.target_speed_mps == pytest.approx(2.0)


def test_left_curve_commands_positive_ros_steering():
    """A path bending left should command positive ROS steering. / 向左弯曲路径应产生正ROS转向。"""
    path = np.asarray([[4.0, 0.5], [8.0, 2.0], [12.0, 4.5]], dtype=np.float32)

    command = pure_pursuit_control(
        path,
        vehicle_position=np.asarray([0.0, 0.0]),
        forward_direction=np.asarray([1.0, 0.0]),
    )

    assert command.valid
    assert 0.0 < command.steering_normalized <= 1.0
    assert command.curvature_per_m > 0.0
    assert command.target_speed_mps < 2.0


def test_rotated_vehicle_tracks_global_y_path():
    """Heading rotation should not change a centred command. / 车辆朝向旋转后居中指令不应改变。"""
    path = np.asarray([[0.0, 4.0], [0.0, 8.0], [0.0, 12.0]], dtype=np.float32)

    command = pure_pursuit_control(
        path,
        vehicle_position=np.asarray([0.0, 0.0]),
        forward_direction=np.asarray([0.0, 1.0]),
    )

    assert command.valid
    assert command.steering_normalized == pytest.approx(0.0, abs=1e-7)


def test_missing_or_behind_path_stops_safely():
    """No usable forward path should request a stop. / 没有可用前向路径时应安全停车。"""
    empty = pure_pursuit_control(
        np.empty((0, 2), dtype=np.float32),
        np.asarray([0.0, 0.0]),
        np.asarray([1.0, 0.0]),
    )
    behind = pure_pursuit_control(
        np.asarray([[-8.0, 0.0], [-4.0, 0.0]], dtype=np.float32),
        np.asarray([0.0, 0.0]),
        np.asarray([1.0, 0.0]),
    )

    assert not empty.valid
    assert not behind.valid
    assert empty.target_speed_mps == 0.0
    assert behind.target_speed_mps == 0.0


def test_zero_heading_is_rejected():
    """A zero heading vector is invalid. / 零朝向向量无效。"""
    with pytest.raises(ValueError):
        pure_pursuit_control(
            np.asarray([[4.0, 0.0], [8.0, 0.0]], dtype=np.float32),
            np.asarray([0.0, 0.0]),
            np.asarray([0.0, 0.0]),
        )


def test_single_gate_recovery_can_be_enabled():
    """H4 may crawl toward one visible gate. / H4可低速驶向唯一可见门。"""
    path = np.asarray([[4.0, 0.5]], dtype=np.float32)
    default_control = pure_pursuit_control(
        path,
        np.asarray([0.0, 0.0], dtype=np.float32),
        np.asarray([1.0, 0.0], dtype=np.float32),
    )
    recovery_control = pure_pursuit_control(
        path,
        np.asarray([0.0, 0.0], dtype=np.float32),
        np.asarray([1.0, 0.0], dtype=np.float32),
        minimum_ahead_points=1,
    )
    assert not default_control.valid
    assert recovery_control.valid
    assert recovery_control.target_point.tolist() == [4.0, 0.5]


def test_shorter_lookahead_reduces_heading_lag_on_tight_curve():
    """A tight curve should use a nearer target and stronger early steering.

    急弯应使用更近的目标点，并更早给出较强转向。
    """
    path = np.asarray(
        [[3.0, 0.8], [5.0, 2.2], [7.0, 4.8]],
        dtype=np.float32,
    )
    short = pure_pursuit_control(
        path,
        np.asarray([0.0, 0.0], dtype=np.float32),
        np.asarray([1.0, 0.0], dtype=np.float32),
        lookahead_distance_m=3.0,
    )
    long = pure_pursuit_control(
        path,
        np.asarray([0.0, 0.0], dtype=np.float32),
        np.asarray([1.0, 0.0], dtype=np.float32),
        lookahead_distance_m=5.0,
    )

    assert short.lookahead_distance_m < long.lookahead_distance_m
    assert short.steering_normalized > long.steering_normalized
