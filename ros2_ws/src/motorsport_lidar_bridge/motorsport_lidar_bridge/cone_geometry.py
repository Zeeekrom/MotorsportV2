"""Small geometric cone filter. / 简单的锥桶几何筛选器。"""

from __future__ import annotations

import numpy as np


# Cone limits shared with the Windows prototype. / 与 Windows 原型相同的锥桶范围。
MIN_CONE_POINTS = 8
MAX_CONE_POINTS = 300
MIN_CONE_HORIZONTAL_SPAN = 0.30
MAX_CONE_HORIZONTAL_SPAN = 0.85
MIN_CONE_SECONDARY_SPAN = 0.08
MIN_CONE_HEIGHT = 0.20
MAX_CONE_HEIGHT = 1.10
MIN_CONE_TAPER_RATIO = 0.55
MAX_CONE_TAPER_RATIO = 3.00


def measure_cone_cluster(cluster: np.ndarray) -> dict[str, float]:
    """Measure size and taper. / 测量点簇大小和锥形。"""
    # Measure x, y, and z spans. / 计算 x、y、z 三轴跨度。
    size = cluster.max(axis=0) - cluster.min(axis=0)
    horizontal_max = float(max(size[0], size[1]))
    horizontal_min = float(min(size[0], size[1]))
    height = float(size[2])

    # Compare the lower and upper halves. / 比较点簇的下半部和上半部。
    z_middle = float(cluster[:, 2].min() + height * 0.5)
    lower = cluster[cluster[:, 2] <= z_middle]
    upper = cluster[cluster[:, 2] > z_middle]
    xy_center = cluster[:, :2].mean(axis=0)

    lower_radius = (
        float(np.mean(np.linalg.norm(lower[:, :2] - xy_center, axis=1)))
        if len(lower)
        else 0.0
    )
    upper_radius = (
        float(np.mean(np.linalg.norm(upper[:, :2] - xy_center, axis=1)))
        if len(upper)
        else 0.0
    )

    return {
        "point_count": float(len(cluster)),
        "horizontal_max": horizontal_max,
        "horizontal_min": horizontal_min,
        "height": height,
        "taper_ratio": lower_radius / max(upper_radius, 0.01),
    }


def is_cone_cluster(cluster: np.ndarray) -> tuple[bool, dict[str, float]]:
    """Accept a cone-like cluster. / 接受形状像锥桶的点簇。"""
    features = measure_cone_cluster(cluster)
    accepted = (
        MIN_CONE_POINTS <= features["point_count"] <= MAX_CONE_POINTS
        and MIN_CONE_HORIZONTAL_SPAN
        <= features["horizontal_max"]
        <= MAX_CONE_HORIZONTAL_SPAN
        and features["horizontal_min"] >= MIN_CONE_SECONDARY_SPAN
        and MIN_CONE_HEIGHT <= features["height"] <= MAX_CONE_HEIGHT
        and MIN_CONE_TAPER_RATIO
        <= features["taper_ratio"]
        <= MAX_CONE_TAPER_RATIO
    )
    return accepted, features
