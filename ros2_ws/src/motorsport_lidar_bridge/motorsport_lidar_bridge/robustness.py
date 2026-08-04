"""Deterministic sensor faults and obstacle safety. / 确定性传感器故障与障碍安全。"""

from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np


@dataclass(frozen=True)
class SensorFaultProfile:
    """One repeatable point-detection fault profile. / 一组可重复的点检测故障参数。"""

    dropout_rate: float = 0.0
    noise_standard_deviation_m: float = 0.0
    seed: int = 1

    def __post_init__(self) -> None:
        if not 0.0 <= float(self.dropout_rate) < 1.0:
            raise ValueError(
                "dropout_rate must be in [0, 1). / "
                "dropout_rate必须在[0, 1)范围内。"
            )
        if float(self.noise_standard_deviation_m) < 0.0:
            raise ValueError(
                "noise_standard_deviation_m must be non-negative. / "
                "noise_standard_deviation_m不能为负数。"
            )


@dataclass(frozen=True)
class FaultInjectionResult:
    """Perturbed points and accounting data. / 扰动后的点及统计信息。"""

    points: np.ndarray
    input_count: int
    output_count: int
    dropped_count: int


class DeterministicSensorFaultInjector:
    """Apply seeded dropout and Gaussian position noise. / 注入固定种子的漏检与高斯位置噪声。"""

    def __init__(self, profile: SensorFaultProfile) -> None:
        self.profile = profile
        self.random = np.random.default_rng(int(profile.seed))
        self.call_count = 0

    def apply(self, points: np.ndarray) -> FaultInjectionResult:
        """Perturb one observation while preserving its column shape. / 扰动一次观测并保留列结构。"""
        array = np.asarray(points, dtype=np.float32)
        if array.ndim != 2:
            raise ValueError(
                "points must be a two-dimensional array. / points必须是二维数组。"
            )
        self.call_count += 1
        input_count = len(array)
        if input_count == 0:
            return FaultInjectionResult(
                points=array.copy(),
                input_count=0,
                output_count=0,
                dropped_count=0,
            )

        # Draw the keep mask before noise so equal seeds reproduce all calls.
        # 先生成保留掩码再加噪声，使相同种子能复现完整调用序列。
        keep = self.random.random(input_count) >= float(
            self.profile.dropout_rate
        )
        perturbed = array[keep].copy()
        coordinate_count = min(3, perturbed.shape[1])
        noise_sigma = float(self.profile.noise_standard_deviation_m)
        if noise_sigma > 0.0 and len(perturbed):
            noise = self.random.normal(
                0.0,
                noise_sigma,
                size=(len(perturbed), coordinate_count),
            )
            perturbed[:, :coordinate_count] += noise.astype(np.float32)
        return FaultInjectionResult(
            points=np.ascontiguousarray(perturbed, dtype=np.float32),
            input_count=input_count,
            output_count=len(perturbed),
            dropped_count=input_count - len(perturbed),
        )


@dataclass(frozen=True)
class SafetyDecision:
    """Speed limit and safety state for one control cycle. / 单次控制周期的限速与安全状态。"""

    state: str
    requested_speed_mps: float
    safe_speed_mps: float
    obstacle_distance_m: float


class DynamicObstacleSafety:
    """Slow, stop, and resume with stop-state hysteresis. / 使用停车迟滞实现减速、停车与恢复。"""

    def __init__(
        self,
        *,
        slow_distance_m: float = 8.0,
        stop_distance_m: float = 3.0,
        resume_distance_m: float = 5.0,
        stop_trigger_buffer_m: float = 0.3,
    ) -> None:
        if not (
            0.0 < stop_distance_m < resume_distance_m < slow_distance_m
        ):
            raise ValueError(
                "Expected 0 < stop < resume < slow distance. / "
                "距离必须满足0 < 停车 < 恢复 < 减速。"
            )
        if not (
            0.0 <= stop_trigger_buffer_m
            < resume_distance_m - stop_distance_m
        ):
            raise ValueError(
                "stop_trigger_buffer_m must fit below resume distance. / "
                "stop_trigger_buffer_m必须小于停车到恢复距离的间隔。"
            )
        self.slow_distance_m = float(slow_distance_m)
        self.stop_distance_m = float(stop_distance_m)
        self.resume_distance_m = float(resume_distance_m)
        self.stop_trigger_distance_m = float(
            stop_distance_m + stop_trigger_buffer_m
        )
        self.stopped = False

    def update(
        self,
        requested_speed_mps: float,
        obstacle_distance_m: float | None,
    ) -> SafetyDecision:
        """Limit speed from the nearest forward obstacle. / 根据最近前方障碍限制速度。"""
        requested_speed = max(0.0, float(requested_speed_mps))
        has_obstacle = (
            obstacle_distance_m is not None
            and math.isfinite(float(obstacle_distance_m))
            and float(obstacle_distance_m) >= 0.0
        )
        if not has_obstacle:
            self.stopped = False
            return SafetyDecision(
                state="clear",
                requested_speed_mps=requested_speed,
                safe_speed_mps=requested_speed,
                obstacle_distance_m=math.inf,
            )

        distance = float(obstacle_distance_m)
        # Trigger slightly before the minimum clearance. A proportional speed
        # command otherwise approaches the exact threshold asymptotically.
        # 在最小净空前稍早触发；否则比例速度指令只会无限接近精确阈值。
        if distance <= self.stop_trigger_distance_m:
            self.stopped = True
        elif self.stopped and distance <= self.resume_distance_m:
            # Do not oscillate between stop and crawl near the threshold.
            # 不在阈值附近反复切换停车与慢行。
            return SafetyDecision(
                state="stopped",
                requested_speed_mps=requested_speed,
                safe_speed_mps=0.0,
                obstacle_distance_m=distance,
            )
        else:
            self.stopped = False

        if self.stopped:
            state = "stopped"
            safe_speed = 0.0
        elif distance < self.slow_distance_m:
            state = "slow"
            scale = (
                (distance - self.stop_distance_m)
                / (self.slow_distance_m - self.stop_distance_m)
            )
            safe_speed = requested_speed * float(np.clip(scale, 0.0, 1.0))
        else:
            state = "clear"
            safe_speed = requested_speed
        return SafetyDecision(
            state=state,
            requested_speed_mps=requested_speed,
            safe_speed_mps=safe_speed,
            obstacle_distance_m=distance,
        )


def forward_obstacle_distance(
    track_length_m: float,
    vehicle_progress_m: float,
    obstacle_progress_m: float | None,
) -> float | None:
    """Return wrapped forward arc distance to an active obstacle. / 返回到活动障碍的循环前向弧长。"""
    track_length = float(track_length_m)
    if track_length <= 0.0:
        raise ValueError(
            "track_length_m must be positive. / track_length_m必须为正数。"
        )
    if obstacle_progress_m is None:
        return None
    return (
        float(obstacle_progress_m) - float(vehicle_progress_m)
    ) % track_length
