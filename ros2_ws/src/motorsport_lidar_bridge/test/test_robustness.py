"""Tests for sensor faults and obstacle safety. / 传感器故障与障碍安全测试。"""

import math

import numpy as np
import pytest

from motorsport_lidar_bridge.robustness import (
    DeterministicSensorFaultInjector,
    DynamicObstacleSafety,
    SensorFaultProfile,
    forward_obstacle_distance,
)


def test_fault_injection_is_reproducible():
    """Equal seeds produce equal dropout and noise. / 相同种子产生相同漏检与噪声。"""
    points = np.column_stack(
        (
            np.linspace(0.0, 10.0, 100),
            np.zeros(100),
            np.ones(100),
        )
    ).astype(np.float32)
    profile = SensorFaultProfile(
        dropout_rate=0.25,
        noise_standard_deviation_m=0.10,
        seed=42,
    )

    first = DeterministicSensorFaultInjector(profile).apply(points)
    second = DeterministicSensorFaultInjector(profile).apply(points)

    assert first.dropped_count == second.dropped_count
    assert 10 <= first.dropped_count <= 40
    assert np.array_equal(first.points, second.points)
    assert not np.allclose(first.points[:, 0], points[: len(first.points), 0])


def test_zero_fault_profile_preserves_points():
    """A zero profile is an exact pass-through. / 零故障参数会完全保留点。"""
    points = np.asarray([[1.0, 2.0], [3.0, 4.0]], dtype=np.float32)

    result = DeterministicSensorFaultInjector(
        SensorFaultProfile()
    ).apply(points)

    assert result.dropped_count == 0
    assert np.array_equal(result.points, points)


def test_invalid_fault_profile_is_rejected():
    """Invalid probabilities and noise fail clearly. / 无效概率和噪声会明确失败。"""
    with pytest.raises(ValueError):
        SensorFaultProfile(dropout_rate=1.0)
    with pytest.raises(ValueError):
        SensorFaultProfile(dropout_rate=-0.1)
    with pytest.raises(ValueError):
        SensorFaultProfile(noise_standard_deviation_m=-0.01)


def test_safety_layer_slows_stops_and_resumes():
    """The safety state follows clear-slow-stop-resume. / 安全状态按畅通、减速、停车、恢复变化。"""
    safety = DynamicObstacleSafety()

    clear = safety.update(2.0, 12.0)
    slow = safety.update(2.0, 5.5)
    stopped = safety.update(2.0, 3.2)
    held = safety.update(2.0, 4.0)
    resumed = safety.update(2.0, 6.0)

    assert clear.state == "clear"
    assert clear.safe_speed_mps == pytest.approx(2.0)
    assert slow.state == "slow"
    assert 0.0 < slow.safe_speed_mps < 2.0
    assert stopped.state == "stopped"
    assert stopped.safe_speed_mps == 0.0
    assert held.state == "stopped"
    assert held.safe_speed_mps == 0.0
    assert resumed.state == "slow"
    assert resumed.safe_speed_mps > 0.0


def test_no_obstacle_clears_a_held_stop():
    """Removing the obstacle restores the requested speed. / 障碍消失后恢复目标速度。"""
    safety = DynamicObstacleSafety()
    safety.update(2.0, 2.0)

    decision = safety.update(2.0, None)

    assert decision.state == "clear"
    assert decision.safe_speed_mps == pytest.approx(2.0)
    assert math.isinf(decision.obstacle_distance_m)


def test_forward_distance_wraps_at_start_line():
    """Obstacle distance wraps across the lap boundary. / 障碍距离可跨越圈起终点。"""
    distance = forward_obstacle_distance(
        100.0,
        vehicle_progress_m=96.0,
        obstacle_progress_m=4.0,
    )

    assert distance == pytest.approx(8.0)
    assert forward_obstacle_distance(100.0, 10.0, None) is None
    with pytest.raises(ValueError):
        forward_obstacle_distance(0.0, 0.0, 1.0)
