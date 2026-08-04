"""Tests for cone-landmark relocalization. / 锥桶地标重定位测试。"""

import numpy as np

from motorsport_lidar_bridge.localization import (
    LandmarkRelocalizer,
    estimate_translation,
)


def test_translation_is_recovered():
    """Known drift is removed. / 已知漂移可被消除。"""
    reference = np.asarray(
        [[0.0, 0.0], [3.0, 1.0], [6.0, -1.0], [9.0, 2.0]],
        dtype=np.float32,
    )
    drift = np.asarray([0.8, -0.4], dtype=np.float32)
    result = estimate_translation(reference + drift, reference)
    assert result.valid
    assert result.matches == 4
    assert np.allclose(result.translation, -drift, atol=1e-5)
    assert result.corrected_residual_m < 1e-5


def test_outlier_is_rejected():
    """One wrong match does not move the robust estimate. / 一个错误匹配不会拉偏稳健估计。"""
    reference = np.asarray(
        [[0.0, 0.0], [4.0, 0.0], [8.0, 0.0], [12.0, 0.0]],
        dtype=np.float32,
    )
    observed = reference + np.asarray([0.5, 0.2], dtype=np.float32)
    observed = np.vstack((observed, [50.0, 50.0]))
    result = estimate_translation(observed, reference)
    assert result.valid
    assert np.allclose(result.translation, [-0.5, -0.2], atol=1e-5)


def test_too_few_matches_is_invalid():
    """At least three matches are required. / 至少需要三个匹配。"""
    result = estimate_translation(
        np.asarray([[0.2, 0.0], [2.2, 0.0]], dtype=np.float32),
        np.asarray([[0.0, 0.0], [2.0, 0.0]], dtype=np.float32),
    )
    assert not result.valid
    assert result.matches == 0


def test_smoothed_relocalizer_corrects_points():
    """Repeated updates produce a usable point correction. / 连续更新会产生可用的点校正。"""
    reference = np.asarray(
        [[0.0, 0.0], [4.0, 0.0], [8.0, 0.0]],
        dtype=np.float32,
    )
    relocalizer = LandmarkRelocalizer(smoothing=0.5)
    relocalizer.update(reference + [0.4, 0.0], reference)
    relocalizer.update(reference + [0.8, 0.0], reference)
    assert np.allclose(relocalizer.translation, [-0.6, 0.0], atol=1e-5)
    corrected = relocalizer.correct(reference + [0.8, 0.0])
    assert np.allclose(corrected, reference + [0.2, 0.0], atol=1e-5)


def test_relocalizer_tracks_drift_beyond_one_match_radius():
    """Incremental updates track large accumulated drift. / 增量更新可跟踪超过单次匹配半径的累计漂移。"""
    reference = np.asarray(
        [[0.0, 0.0], [4.0, 0.0], [8.0, 0.0], [12.0, 0.0]],
        dtype=np.float32,
    )
    relocalizer = LandmarkRelocalizer(smoothing=0.6)

    # Each update moves only 0.2 m, while total drift reaches 4.0 m.
    # 每次只增加0.2米，但累计漂移最终达到4.0米。
    for drift_x in np.linspace(0.2, 4.0, 20):
        result = relocalizer.update(
            reference + [float(drift_x), 0.0],
            reference,
        )
        assert result.valid

    corrected = relocalizer.correct(reference + [4.0, 0.0])
    mean_error = float(np.mean(np.linalg.norm(corrected - reference, axis=1)))
    assert mean_error < 0.2
    assert relocalizer.translation[0] < -3.8
