"""Tests for general closed-track geometry. / 通用闭环赛道几何测试。"""

import math

import numpy as np
import pytest

from motorsport_lidar_bridge.closed_track import (
    PolylineLapTracker,
    SUPPORTED_TRACK_SHAPES,
    generate_closed_track,
    project_to_track,
    sample_at_progress,
    sample_path_ahead,
    sample_ros_path_ahead,
)


@pytest.mark.parametrize("shape", SUPPORTED_TRACK_SHAPES)
def test_supported_tracks_share_start_and_width(shape):
    """Every track starts consistently and preserves width. / 每条赛道起点一致且宽度正确。"""
    track = generate_closed_track(shape)

    assert np.allclose(track.centreline[0], [0.0, 0.0], atol=1e-6)
    assert track.tangents[0, 0] > 0.99
    widths = np.linalg.norm(track.left_cones - track.right_cones, axis=1)
    assert np.allclose(widths, 4.4, atol=1e-5)
    assert track.length_m > 50.0


def test_shapes_have_distinct_lengths():
    """The three layouts are geometrically different. / 三种布局的几何长度不同。"""
    lengths = {
        shape: generate_closed_track(shape).length_m
        for shape in SUPPORTED_TRACK_SHAPES
    }

    assert len({round(length, 1) for length in lengths.values()}) == 3


def test_projection_reports_known_lateral_offset():
    """Projection recovers a known left offset. / 投影可恢复已知左侧偏移。"""
    track = generate_closed_track("oval")
    progress = 0.35 * track.length_m
    centre, tangent = sample_at_progress(track, progress)
    left_normal = np.asarray([-tangent[1], tangent[0]], dtype=np.float32)

    projection = project_to_track(track, centre + 0.6 * left_normal)

    assert projection.cross_track_error_m == pytest.approx(0.6, abs=0.03)
    assert projection.signed_cross_track_error_m > 0.0


def test_multi_lap_tracker_unwraps_three_laps():
    """Progress remains continuous across three start-line crossings. / 进度跨越三次起终点后仍连续。"""
    track = generate_closed_track("technical")
    tracker = PolylineLapTracker(track)
    result = None

    for progress in np.linspace(0.0, 3.02 * track.length_m, 500):
        position, tangent = sample_at_progress(track, float(progress))
        heading = math.degrees(math.atan2(tangent[1], tangent[0]))
        result = tracker.update(position, heading)

    assert result is not None
    assert result.completed_laps == 3
    assert result.progress_m == pytest.approx(
        3.02 * track.length_m,
        rel=0.01,
    )
    assert result.cross_track_error_m < 1e-4
    assert abs(result.heading_error_degrees) < 1e-3


def test_ahead_path_wraps_across_start_line():
    """Forward samples remain ordered across the start line. / 前向采样跨起终点后仍保持有序。"""
    track = generate_closed_track("circle")
    progress = track.length_m - 3.0
    path = sample_path_ahead(
        track,
        progress,
        horizon_m=10.0,
        spacing_m=2.0,
    )

    assert path.shape == (5, 2)
    expected_first, _ = sample_at_progress(track, progress + 2.0)
    expected_last, _ = sample_at_progress(track, progress + 10.0)
    assert np.allclose(path[0], expected_first)
    assert np.allclose(path[-1], expected_last)


def test_saved_route_reflects_carla_y_into_ros_map():
    """The CARLA-left turn must appear with negative Y in the ROS map.

    CARLA中的左转路线在ROS地图中必须显示为负Y。
    """
    track = generate_closed_track("oval")

    path = sample_ros_path_ahead(
        track,
        np.asarray([0.0, 0.0], dtype=np.float32),
        horizon_m=8.0,
        spacing_m=2.0,
    )

    assert np.all(path[:, 0] > 0.0)
    assert np.all(path[:, 1] < 0.0)


def test_invalid_track_arguments_are_rejected():
    """Invalid geometry inputs fail clearly. / 无效几何输入会明确失败。"""
    with pytest.raises(ValueError):
        generate_closed_track("triangle")
    with pytest.raises(ValueError):
        generate_closed_track("circle", gate_count=8)
    with pytest.raises(ValueError):
        sample_path_ahead(
            generate_closed_track("circle"),
            0.0,
            spacing_m=0.0,
        )
