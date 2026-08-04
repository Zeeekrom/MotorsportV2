"""Landmark-based translation relocalization. / 基于地标的平移重定位。"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class RelocalizationResult:
    """One robust landmark-alignment result. / 一次稳健地标对齐结果。"""

    valid: bool
    translation: np.ndarray
    matches: int
    raw_residual_m: float
    corrected_residual_m: float


def empty_relocalization() -> RelocalizationResult:
    """Return a shape-stable invalid result. / 返回形状固定的无效结果。"""
    return RelocalizationResult(
        valid=False,
        translation=np.zeros(2, dtype=np.float32),
        matches=0,
        raw_residual_m=float("nan"),
        corrected_residual_m=float("nan"),
    )


def estimate_translation(
    observed_points: np.ndarray,
    reference_points: np.ndarray,
    *,
    maximum_match_distance_m: float = 1.8,
    minimum_matches: int = 3,
    outlier_residual_m: float = 0.45,
) -> RelocalizationResult:
    """Estimate reference-minus-observation translation. / 估计参考点减观测点的平移量。"""
    observed = np.asarray(observed_points, dtype=np.float32)
    reference = np.asarray(reference_points, dtype=np.float32)
    if observed.size == 0 or reference.size == 0:
        return empty_relocalization()
    if observed.ndim != 2 or reference.ndim != 2:
        raise ValueError(
            "Point arrays must be two-dimensional. / 点数组必须是二维数组。"
        )
    observed = observed[:, :2]
    reference = reference[:, :2]
    observed = observed[np.isfinite(observed).all(axis=1)]
    reference = reference[np.isfinite(reference).all(axis=1)]
    if not len(observed) or not len(reference):
        return empty_relocalization()

    # Match every observation to its nearest persistent landmark.
    # 把每个观测与最近的持久地标匹配。
    differences = reference[None, :, :] - observed[:, None, :]
    distances = np.linalg.norm(differences, axis=2)
    nearest_reference = np.argmin(distances, axis=1)
    nearest_distance = distances[
        np.arange(len(observed)),
        nearest_reference,
    ]
    accepted = nearest_distance <= float(maximum_match_distance_m)
    if np.count_nonzero(accepted) < int(minimum_matches):
        return empty_relocalization()
    offsets = differences[
        np.arange(len(observed)),
        nearest_reference,
    ][accepted]

    # Median translation is resistant to occasional wrong cone matches.
    # 中位数平移能抵抗偶发的错误锥桶匹配。
    initial_translation = np.median(offsets, axis=0)
    residuals = np.linalg.norm(offsets - initial_translation, axis=1)
    inliers = residuals <= float(outlier_residual_m)
    if np.count_nonzero(inliers) < int(minimum_matches):
        return empty_relocalization()
    inlier_offsets = offsets[inliers]
    translation = np.median(inlier_offsets, axis=0).astype(
        np.float32,
    )
    raw_residual = float(np.mean(np.linalg.norm(inlier_offsets, axis=1)))
    corrected_residual = float(
        np.mean(np.linalg.norm(inlier_offsets - translation, axis=1))
    )
    return RelocalizationResult(
        valid=True,
        translation=translation,
        matches=len(inlier_offsets),
        raw_residual_m=raw_residual,
        corrected_residual_m=corrected_residual,
    )


class LandmarkRelocalizer:
    """Smooth repeated landmark alignment updates. / 平滑连续的地标对齐更新。"""

    def __init__(self, *, smoothing: float = 0.45) -> None:
        if not 0.0 < smoothing <= 1.0:
            raise ValueError(
                "smoothing must be in (0, 1]. / smoothing必须在(0, 1]范围内。"
            )
        self.smoothing = float(smoothing)
        self.translation = np.zeros(2, dtype=np.float32)
        self.update_count = 0
        self.last_result = empty_relocalization()

    def update(
        self,
        observed_points: np.ndarray,
        reference_points: np.ndarray,
    ) -> RelocalizationResult:
        """Estimate and smooth one correction update. / 估计并平滑一次校正更新。"""
        observed = np.asarray(observed_points, dtype=np.float32)
        # Apply the previous global correction before nearest-neighbour
        # matching. The new estimate is therefore an incremental residual,
        # so slowly growing drift can exceed one raw match radius over many
        # laps without losing localization.
        # 最近邻匹配前先应用上一轮全局校正；新估计因此是增量残差，
        # 即使多圈后的累计漂移超过一次原始匹配半径，也不会丢失定位。
        predicted = observed.copy()
        if predicted.size:
            predicted[:, :2] += self.translation
        residual = estimate_translation(predicted, reference_points)
        if not residual.valid:
            self.last_result = residual
            return residual
        if self.update_count == 0:
            self.translation = residual.translation.copy()
        else:
            self.translation = (
                self.translation + self.smoothing * residual.translation
            ).astype(np.float32)
        self.update_count += 1
        self.last_result = RelocalizationResult(
            valid=True,
            translation=self.translation.copy(),
            matches=residual.matches,
            raw_residual_m=residual.raw_residual_m,
            corrected_residual_m=residual.corrected_residual_m,
        )
        return self.last_result

    def correct(self, points: np.ndarray) -> np.ndarray:
        """Apply the current planar correction to points. / 把当前平面校正应用到点。"""
        array = np.asarray(points, dtype=np.float32)
        corrected = array.copy()
        if corrected.size:
            corrected[:, :2] += self.translation
        return corrected
