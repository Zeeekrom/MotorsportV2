"""Tests for circular lap truth metrics. / 圆形赛道圈真值指标测试。"""

import math

import numpy as np
import pytest

from motorsport_lidar_bridge.lap_tracking import CircularLapTracker


def circle_position(radius: float, angle: float) -> np.ndarray:
    """Create one point on the test circle. / 创建测试圆上的一个点。"""
    return np.asarray(
        [radius * math.sin(angle), radius * (1.0 - math.cos(angle))],
        dtype=np.float32,
    )


def test_progress_unwraps_across_start_line():
    """Progress stays increasing when angle wraps. / 角度跨界时进度仍保持递增。"""
    tracker = CircularLapTracker(10.0)
    result = None
    for angle in np.linspace(0.0, 2.0 * math.pi, 65):
        result = tracker.update(
            circle_position(10.0, float(angle)),
            math.degrees(float(angle)),
        )
    assert result is not None
    assert result.progress_percent == pytest.approx(100.0, abs=0.01)
    assert result.completed_laps == 1
    assert result.distance_to_start_m < 1e-4


def test_cross_track_and_heading_errors():
    """Offset and heading error are reported. / 可报告横向偏移和航向误差。"""
    tracker = CircularLapTracker(10.0)
    result = tracker.update(
        np.asarray([11.0, 10.0], dtype=np.float32),
        100.0,
    )
    assert result.cross_track_error_m == pytest.approx(1.0)
    assert result.heading_error_degrees == pytest.approx(10.0)


def test_invalid_radius_is_rejected():
    """A non-positive radius is invalid. / 非正半径无效。"""
    with pytest.raises(ValueError):
        CircularLapTracker(0.0)
