"""Tests for cone geometry. / 锥桶几何规则测试。"""

import numpy as np

from motorsport_lidar_bridge.cone_geometry import is_cone_cluster


def test_cone_shape_is_accepted():
    """A tapered cluster should pass. / 锥形点簇应该通过。"""
    points = []
    for z, radius in zip(np.linspace(0.0, 0.7, 6), np.linspace(0.28, 0.08, 6)):
        for angle in np.linspace(0.0, 2.0 * np.pi, 12, endpoint=False):
            points.append([radius * np.cos(angle), radius * np.sin(angle), z])
    accepted, _ = is_cone_cluster(np.asarray(points, dtype=np.float32))
    assert accepted


def test_flat_cluster_is_rejected():
    """A flat object should fail. / 扁平物体应该被拒绝。"""
    x = np.linspace(-0.3, 0.3, 10)
    points = np.column_stack((x, np.zeros_like(x), np.zeros_like(x)))
    accepted, _ = is_cone_cluster(points.astype(np.float32))
    assert not accepted
