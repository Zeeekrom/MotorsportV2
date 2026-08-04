"""Tests for the cone centre-path baseline. / 锥桶中心路径基线测试。"""

import numpy as np
import pytest

from motorsport_lidar_bridge.path_planning import (
    ConeMapAccumulator,
    plan_center_path,
)


def test_straight_channel_produces_ordered_centreline():
    """Five shuffled cone pairs should form a centred path. / 五对乱序锥桶应形成居中路径。"""
    cones = np.asarray(
        [
            [14.0, -2.2],
            [6.0, 2.2],
            [22.0, -2.2],
            [10.0, 2.2],
            [18.0, 2.2],
            [6.0, -2.2],
            [14.0, 2.2],
            [22.0, 2.2],
            [10.0, -2.2],
            [18.0, -2.2],
        ],
        dtype=np.float32,
    )

    path = plan_center_path(cones)

    np.testing.assert_allclose(path.points[:, 0], [6, 10, 14, 18, 22])
    np.testing.assert_allclose(path.points[:, 1], 0.0, atol=1e-6)
    np.testing.assert_allclose(path.widths, 4.4, atol=1e-6)


def test_small_longitudinal_offsets_are_paired():
    """Slightly staggered rows should still pair correctly. / 轻微错位的两排锥桶仍应正确配对。"""
    cones = np.asarray(
        [
            [6.0, 2.0],
            [10.0, 2.1],
            [14.0, 2.2],
            [6.3, -2.1],
            [9.8, -2.0],
            [14.2, -2.2],
        ],
        dtype=np.float32,
    )

    path = plan_center_path(cones)

    assert len(path.points) == 3
    assert np.all(np.diff(path.points[:, 0]) > 0.0)
    assert np.all((path.widths > 4.0) & (path.widths < 4.5))


def test_one_sided_or_implausible_width_returns_no_path():
    """One side or an unsafe width must not create a path. / 单侧锥桶或异常宽度不能生成路径。"""
    one_side = np.asarray([[6.0, 2.0], [10.0, 2.0]], dtype=np.float32)
    too_narrow = np.asarray(
        [[6.0, 0.4], [10.0, 0.4], [6.0, -0.4], [10.0, -0.4]],
        dtype=np.float32,
    )

    assert len(plan_center_path(one_side).points) == 0
    assert len(plan_center_path(too_narrow).points) == 0


def test_bad_input_shape_is_rejected():
    """Malformed arrays should fail clearly. / 形状错误的数组应给出明确错误。"""
    with pytest.raises(ValueError):
        plan_center_path(np.asarray([1.0, 2.0], dtype=np.float32))


def test_rotated_vehicle_heading_changes_pairing_axes():
    """A north-facing vehicle should plan along global Y. / 朝北车辆应沿全局Y方向规划。"""
    cones = np.asarray(
        [
            [-2.2, 6.0],
            [2.2, 6.0],
            [-2.2, 10.0],
            [2.2, 10.0],
            [-2.2, 14.0],
            [2.2, 14.0],
        ],
        dtype=np.float32,
    )

    path = plan_center_path(cones, forward_direction=(0.0, 1.0))

    np.testing.assert_allclose(path.points[:, 0], 0.0, atol=1e-6)
    np.testing.assert_allclose(path.points[:, 1], [6.0, 10.0, 14.0])


def test_curved_corridor_uses_current_tangent_direction():
    """A curved local corridor should retain ordered midpoints. / 弯曲局部通道应保留有序中点。"""
    x_values = np.asarray([8.0, 12.0, 16.0, 20.0], dtype=np.float32)
    centre_points = []
    cones = []
    curvature = 0.012
    for x_value in x_values:
        centre = np.asarray(
            [x_value, curvature * x_value**2],
            dtype=np.float32,
        )
        tangent = np.asarray(
            [1.0, 2.0 * curvature * x_value],
            dtype=np.float32,
        )
        tangent /= np.linalg.norm(tangent)
        normal = np.asarray([-tangent[1], tangent[0]], dtype=np.float32)
        centre_points.append(centre)
        cones.extend([centre + normal * 2.2, centre - normal * 2.2])
    vehicle_x = 6.0
    vehicle_position = np.asarray(
        [vehicle_x, curvature * vehicle_x**2],
        dtype=np.float32,
    )
    forward = np.asarray(
        [1.0, 2.0 * curvature * vehicle_x],
        dtype=np.float32,
    )

    path = plan_center_path(
        np.asarray(cones),
        vehicle_position=vehicle_position,
        forward_direction=forward,
    )

    np.testing.assert_allclose(
        path.points,
        np.asarray(centre_points),
        atol=0.15,
    )


def test_cone_map_merges_repeated_observations_and_prunes():
    """Repeated cones should merge and distant landmarks should prune. / 重复锥桶应合并，远处地标应裁剪。"""
    cone_map = ConeMapAccumulator(merge_distance=0.5)
    cone_map.update(
        np.asarray([[5.0, 2.0, 0.0], [5.0, -2.0, 0.0]], dtype=np.float32)
    )
    cone_map.update(
        np.asarray([[5.1, 2.1, 0.0], [5.1, -2.1, 0.0]], dtype=np.float32)
    )

    assert cone_map.size == 2
    assert len(cone_map.points(minimum_observations=2)) == 2
    np.testing.assert_allclose(
        cone_map.points(minimum_observations=2)[:, 0],
        5.05,
        atol=1e-5,
    )

    cone_map.prune(np.asarray([100.0, 100.0]), maximum_distance=10.0)
    assert cone_map.size == 0


def test_locked_cone_map_refines_nearby_landmark_and_rejects_outlier():
    """A prebuilt map may refine landmarks but must not add noise.

    预建地图可以细化已有地标，但不能把噪声添加为新地标。
    """
    cone_map = ConeMapAccumulator(merge_distance=0.5)
    cone_map.update(
        np.asarray([[5.0, 2.0, 0.0], [5.0, -2.0, 0.0]], dtype=np.float32)
    )

    cone_map.update(
        np.asarray([[5.2, 2.1, 0.0], [20.0, 20.0, 0.0]], dtype=np.float32),
        allow_new=False,
    )

    assert cone_map.size == 2
    np.testing.assert_allclose(
        cone_map.points()[0],
        [5.1, 2.05, 0.0],
        atol=1e-5,
    )
