"""Deterministic H5 multi-lap robustness benchmark. / 确定性H5多圈鲁棒性基准。"""

from __future__ import annotations

from dataclasses import asdict, dataclass
import math

import numpy as np

from .closed_track import (
    PolylineLapTracker,
    SUPPORTED_TRACK_SHAPES,
    generate_closed_track,
    project_to_track,
    sample_path_ahead,
)
from .localization import LandmarkRelocalizer
from .path_controller import pure_pursuit_control
from .robustness import (
    DeterministicSensorFaultInjector,
    DynamicObstacleSafety,
    SensorFaultProfile,
    forward_obstacle_distance,
)


@dataclass(frozen=True)
class ScenarioConfig:
    """One H5 track and fault combination. / 一组H5赛道与故障组合。"""

    name: str
    track_shape: str
    target_laps: int = 3
    dropout_rate: float = 0.0
    noise_standard_deviation_m: float = 0.02
    localization_drift_per_m: float = 0.012
    dynamic_obstacle: bool = False
    requested_speed_mps: float = 2.2
    seed: int = 1
    maximum_time_s: float = 260.0

    def __post_init__(self) -> None:
        if self.track_shape not in SUPPORTED_TRACK_SHAPES:
            raise ValueError(
                f"Unsupported track_shape: {self.track_shape}. / "
                f"不支持的track_shape：{self.track_shape}。"
            )
        if int(self.target_laps) < 1:
            raise ValueError(
                "target_laps must be positive. / target_laps必须为正数。"
            )
        if float(self.localization_drift_per_m) < 0.0:
            raise ValueError(
                "localization_drift_per_m must be non-negative. / "
                "localization_drift_per_m不能为负数。"
            )
        if float(self.requested_speed_mps) <= 0.0:
            raise ValueError(
                "requested_speed_mps must be positive. / "
                "requested_speed_mps必须为正数。"
            )
        if float(self.maximum_time_s) <= 0.0:
            raise ValueError(
                "maximum_time_s must be positive. / maximum_time_s必须为正数。"
            )
        SensorFaultProfile(
            dropout_rate=self.dropout_rate,
            noise_standard_deviation_m=self.noise_standard_deviation_m,
            seed=self.seed,
        )


def percentile(values: list[float], fraction: float) -> float:
    """Calculate a dependency-free linear percentile. / 计算无额外依赖的线性百分位数。"""
    ordered = sorted(float(value) for value in values)
    if not ordered:
        return math.nan
    position = (len(ordered) - 1) * float(fraction)
    lower = int(math.floor(position))
    upper = int(math.ceil(position))
    if lower == upper:
        return ordered[lower]
    weight = position - lower
    return ordered[lower] * (1.0 - weight) + ordered[upper] * weight


def mean(values: list[float]) -> float:
    """Return a safe arithmetic mean. / 返回安全的算术平均值。"""
    return sum(values) / len(values) if values else math.nan


def default_scenarios(*, target_laps: int = 3) -> list[ScenarioConfig]:
    """Return the standard H5 evaluation matrix. / 返回标准H5评估矩阵。"""
    return [
        ScenarioConfig(
            name="circle_nominal",
            track_shape="circle",
            target_laps=target_laps,
            seed=101,
        ),
        ScenarioConfig(
            name="oval_nominal",
            track_shape="oval",
            target_laps=target_laps,
            seed=102,
        ),
        ScenarioConfig(
            name="technical_nominal",
            track_shape="technical",
            target_laps=target_laps,
            seed=103,
        ),
        ScenarioConfig(
            name="circle_moderate_faults",
            track_shape="circle",
            target_laps=target_laps,
            dropout_rate=0.20,
            noise_standard_deviation_m=0.10,
            localization_drift_per_m=0.015,
            seed=201,
        ),
        ScenarioConfig(
            name="oval_moderate_faults",
            track_shape="oval",
            target_laps=target_laps,
            dropout_rate=0.20,
            noise_standard_deviation_m=0.10,
            localization_drift_per_m=0.015,
            seed=202,
        ),
        ScenarioConfig(
            name="technical_moderate_faults",
            track_shape="technical",
            target_laps=target_laps,
            dropout_rate=0.20,
            noise_standard_deviation_m=0.10,
            localization_drift_per_m=0.015,
            seed=203,
        ),
        ScenarioConfig(
            name="technical_stress_faults",
            track_shape="technical",
            target_laps=target_laps,
            dropout_rate=0.35,
            noise_standard_deviation_m=0.18,
            localization_drift_per_m=0.020,
            requested_speed_mps=2.0,
            seed=301,
        ),
        ScenarioConfig(
            name="oval_dynamic_obstacle",
            track_shape="oval",
            target_laps=target_laps,
            dropout_rate=0.15,
            noise_standard_deviation_m=0.08,
            localization_drift_per_m=0.015,
            dynamic_obstacle=True,
            seed=401,
            maximum_time_s=280.0,
        ),
    ]


def _visible_landmarks(
    landmarks: np.ndarray,
    vehicle_position: np.ndarray,
    forward_direction: np.ndarray,
) -> np.ndarray:
    """Select a practical forward LiDAR neighbourhood. / 选择实用的前向雷达邻域。"""
    relative = landmarks - vehicle_position
    distances = np.linalg.norm(relative, axis=1)
    longitudinal = relative @ forward_direction
    visible = (distances <= 26.0) & (longitudinal >= -4.0)
    return landmarks[visible]


def _scenario_checks(
    config: ScenarioConfig,
    metrics: dict[str, int | float | bool],
) -> tuple[dict[str, int | float | bool], dict[str, bool]]:
    """Build scenario-specific acceptance criteria. / 建立场景专用验收标准。"""
    criteria: dict[str, int | float | bool] = {
        "minimum_completed_laps": int(config.target_laps),
        "maximum_final_distance_to_start_m": 3.0,
        "maximum_mean_cross_track_error_m": 1.0,
        "maximum_p95_cross_track_error_m": 1.5,
        "maximum_max_cross_track_error_m": 2.5,
        "maximum_p95_heading_error_degrees": 18.0,
        "maximum_mean_corrected_localization_error_m": 0.45,
        "maximum_p95_corrected_localization_error_m": 0.75,
        "minimum_localization_improvement_percent": 40.0,
        "minimum_relocalization_updates": 20 * int(config.target_laps),
        "minimum_sensor_retention_ratio": max(
            0.50,
            1.0 - float(config.dropout_rate) - 0.08,
        ),
    }
    checks = {
        "minimum_completed_laps": int(metrics["completed_laps"])
        >= int(criteria["minimum_completed_laps"]),
        "maximum_final_distance_to_start_m": float(
            metrics["final_distance_to_start_m"]
        )
        <= float(criteria["maximum_final_distance_to_start_m"]),
        "maximum_mean_cross_track_error_m": float(
            metrics["mean_cross_track_error_m"]
        )
        <= float(criteria["maximum_mean_cross_track_error_m"]),
        "maximum_p95_cross_track_error_m": float(
            metrics["p95_cross_track_error_m"]
        )
        <= float(criteria["maximum_p95_cross_track_error_m"]),
        "maximum_max_cross_track_error_m": float(
            metrics["maximum_cross_track_error_m"]
        )
        <= float(criteria["maximum_max_cross_track_error_m"]),
        "maximum_p95_heading_error_degrees": float(
            metrics["p95_heading_error_degrees"]
        )
        <= float(criteria["maximum_p95_heading_error_degrees"]),
        "maximum_mean_corrected_localization_error_m": float(
            metrics["mean_corrected_localization_error_m"]
        )
        <= float(criteria["maximum_mean_corrected_localization_error_m"]),
        "maximum_p95_corrected_localization_error_m": float(
            metrics["p95_corrected_localization_error_m"]
        )
        <= float(criteria["maximum_p95_corrected_localization_error_m"]),
        "minimum_localization_improvement_percent": float(
            metrics["localization_improvement_percent"]
        )
        >= float(criteria["minimum_localization_improvement_percent"]),
        "minimum_relocalization_updates": int(
            metrics["relocalization_updates"]
        )
        >= int(criteria["minimum_relocalization_updates"]),
        "minimum_sensor_retention_ratio": float(
            metrics["sensor_retention_ratio"]
        )
        >= float(criteria["minimum_sensor_retention_ratio"]),
    }
    if config.dynamic_obstacle:
        criteria.update(
            {
                "minimum_stopped_duration_s": 0.5,
                "minimum_obstacle_clearance_m": 2.0,
                "must_resume_after_obstacle": True,
            }
        )
        checks.update(
            {
                "minimum_stopped_duration_s": float(
                    metrics["stopped_duration_s"]
                )
                >= float(criteria["minimum_stopped_duration_s"]),
                "minimum_obstacle_clearance_m": float(
                    metrics["minimum_obstacle_distance_m"]
                )
                >= float(criteria["minimum_obstacle_clearance_m"]),
                "must_resume_after_obstacle": bool(
                    metrics["resumed_after_obstacle"]
                ),
            }
        )
    return criteria, checks


def run_scenario(
    config: ScenarioConfig,
    *,
    time_step_s: float = 0.05,
    log_every_n_steps: int = 20,
) -> tuple[list[dict[str, int | float | str]], dict[str, object]]:
    """Run one deterministic perception-localization-control scenario. / 运行一个确定性感知定位控制场景。"""
    track = generate_closed_track(config.track_shape)
    landmarks = np.vstack((track.left_cones, track.right_cones)).astype(
        np.float32
    )
    tracker = PolylineLapTracker(track)
    relocalizer = LandmarkRelocalizer(smoothing=0.45)
    injector = DeterministicSensorFaultInjector(
        SensorFaultProfile(
            dropout_rate=config.dropout_rate,
            noise_standard_deviation_m=(
                config.noise_standard_deviation_m
            ),
            seed=config.seed,
        )
    )
    safety = DynamicObstacleSafety()

    true_position = track.centreline[0, :2].astype(np.float64)
    initial_tangent = track.tangents[0, :2].astype(np.float64)
    heading_rad = math.atan2(initial_tangent[1], initial_tangent[0])
    speed_mps = 0.0
    travelled_distance_m = 0.0
    elapsed_s = 0.0
    detection_interval_steps = 5
    maximum_steps = int(math.ceil(config.maximum_time_s / time_step_s))
    last_detected_count = 0
    last_dropped_count = 0
    last_matches = 0
    total_sensor_inputs = 0
    total_sensor_outputs = 0
    stopped_steps = 0
    obstacle_was_stopped = False
    resumed_after_obstacle = False
    minimum_obstacle_distance_m = math.inf
    obstacle_triggered = False
    obstacle_active = False
    obstacle_release_time_s = math.inf
    obstacle_progress_m = 0.55 * track.length_m
    rows: list[dict[str, int | float | str]] = []
    cross_errors: list[float] = []
    heading_errors: list[float] = []
    raw_errors: list[float] = []
    corrected_errors: list[float] = []
    final_truth = tracker.update(true_position, math.degrees(heading_rad))

    for step in range(maximum_steps):
        elapsed_s = step * time_step_s
        forward = np.asarray(
            [math.cos(heading_rad), math.sin(heading_rad)],
            dtype=np.float64,
        )
        final_truth = tracker.update(
            true_position,
            math.degrees(heading_rad),
        )

        # Add distance-proportional odometry drift in a fixed map direction.
        # 在固定地图方向加入与行驶距离成比例的里程计漂移。
        drift = np.asarray(
            [
                config.localization_drift_per_m * travelled_distance_m,
                -0.5
                * config.localization_drift_per_m
                * travelled_distance_m,
            ],
            dtype=np.float32,
        )
        if step % detection_interval_steps == 0:
            visible = _visible_landmarks(
                landmarks,
                true_position.astype(np.float32),
                forward.astype(np.float32),
            )
            raw_observations = visible + drift
            injected = injector.apply(raw_observations)
            last_detected_count = injected.output_count
            last_dropped_count = injected.dropped_count
            total_sensor_inputs += injected.input_count
            total_sensor_outputs += injected.output_count
            if injected.output_count >= 3:
                result = relocalizer.update(
                    injected.points,
                    landmarks,
                )
                last_matches = result.matches if result.valid else 0

        estimated_position = (
            true_position.astype(np.float32)
            + drift
            + relocalizer.translation
        )
        estimated_projection = project_to_track(
            track,
            estimated_position,
        )
        path = sample_path_ahead(
            track,
            estimated_projection.progress_m,
            horizon_m=24.0,
            spacing_m=2.0,
        )
        command = pure_pursuit_control(
            path,
            estimated_position,
            forward,
            requested_speed_mps=config.requested_speed_mps,
            lookahead_distance_m=4.5,
            minimum_ahead_points=2,
        )

        # Activate one temporary obstacle during the first lap.
        # 在第一圈中激活一个临时障碍。
        if (
            config.dynamic_obstacle
            and not obstacle_triggered
            and final_truth.progress_m >= 0.35 * track.length_m
        ):
            obstacle_triggered = True
            obstacle_active = True
        if obstacle_active and elapsed_s >= obstacle_release_time_s:
            obstacle_active = False
        obstacle_distance = forward_obstacle_distance(
            track.length_m,
            final_truth.progress_m,
            obstacle_progress_m if obstacle_active else None,
        )
        if obstacle_distance is not None:
            minimum_obstacle_distance_m = min(
                minimum_obstacle_distance_m,
                obstacle_distance,
            )
        decision = safety.update(
            command.target_speed_mps if command.valid else 0.0,
            obstacle_distance,
        )
        if decision.state == "stopped":
            stopped_steps += 1
            obstacle_was_stopped = True
            if math.isinf(obstacle_release_time_s):
                # Model an obstacle that clears two seconds after the vehicle
                # has demonstrably stopped. / 模拟车辆确认停车两秒后障碍离开。
                obstacle_release_time_s = elapsed_s + 2.0
        elif (
            obstacle_was_stopped
            and not obstacle_active
            and decision.safe_speed_mps > 0.0
        ):
            resumed_after_obstacle = True

        # Use bounded longitudinal acceleration and a bicycle yaw model.
        # 使用有界纵向加速度和自行车偏航模型。
        target_speed = decision.safe_speed_mps
        acceleration_limit = 1.2 if target_speed >= speed_mps else 3.0
        speed_delta = float(
            np.clip(
                target_speed - speed_mps,
                -acceleration_limit * time_step_s,
                acceleration_limit * time_step_s,
            )
        )
        speed_mps = max(0.0, speed_mps + speed_delta)
        steering = command.steering_normalized if command.valid else 0.0
        steering_angle = math.radians(35.0) * steering
        yaw_rate = speed_mps / 2.875 * math.tan(steering_angle)
        heading_rad += yaw_rate * time_step_s
        true_position += (
            speed_mps
            * np.asarray(
                [math.cos(heading_rad), math.sin(heading_rad)],
                dtype=np.float64,
            )
            * time_step_s
        )
        travelled_distance_m += speed_mps * time_step_s

        raw_error = float(np.linalg.norm(drift))
        corrected_error = float(
            np.linalg.norm(drift + relocalizer.translation)
        )
        cross_errors.append(final_truth.cross_track_error_m)
        heading_errors.append(abs(final_truth.heading_error_degrees))
        raw_errors.append(raw_error)
        corrected_errors.append(corrected_error)

        should_log = (
            step % max(1, int(log_every_n_steps)) == 0
            or final_truth.completed_laps >= config.target_laps
        )
        if should_log:
            rows.append(
                {
                    "scenario": config.name,
                    "track_shape": config.track_shape,
                    "step": step,
                    "time_s": elapsed_s,
                    "completed_laps": final_truth.completed_laps,
                    "lap_progress_percent": (
                        final_truth.lap_progress_percent
                    ),
                    "cross_track_error_m": (
                        final_truth.cross_track_error_m
                    ),
                    "heading_error_degrees": abs(
                        final_truth.heading_error_degrees
                    ),
                    "speed_mps": speed_mps,
                    "requested_speed_mps": (
                        command.target_speed_mps
                        if command.valid
                        else 0.0
                    ),
                    "safe_speed_mps": decision.safe_speed_mps,
                    "steering_normalized": steering,
                    "raw_localization_error_m": raw_error,
                    "corrected_localization_error_m": corrected_error,
                    "relocalization_matches": last_matches,
                    "relocalization_updates": relocalizer.update_count,
                    "detected_landmarks": last_detected_count,
                    "dropped_landmarks": last_dropped_count,
                    "safety_state": decision.state,
                    "obstacle_distance_m": (
                        decision.obstacle_distance_m
                    ),
                    "distance_to_start_m": (
                        final_truth.distance_to_start_m
                    ),
                }
            )
        if final_truth.completed_laps >= config.target_laps:
            break

    mean_raw_error = mean(raw_errors)
    mean_corrected_error = mean(corrected_errors)
    localization_improvement = (
        100.0 * (1.0 - mean_corrected_error / mean_raw_error)
        if mean_raw_error > 1e-9
        else 100.0
    )
    metrics: dict[str, int | float | bool] = {
        "track_length_m": track.length_m,
        "target_laps": int(config.target_laps),
        "completed_laps": int(final_truth.completed_laps),
        "completion_time_s": elapsed_s,
        "travelled_distance_m": travelled_distance_m,
        "observations": len(rows),
        "final_distance_to_start_m": final_truth.distance_to_start_m,
        "mean_cross_track_error_m": mean(cross_errors),
        "p95_cross_track_error_m": percentile(cross_errors, 0.95),
        "maximum_cross_track_error_m": max(cross_errors, default=math.nan),
        "mean_heading_error_degrees": mean(heading_errors),
        "p95_heading_error_degrees": percentile(heading_errors, 0.95),
        "mean_raw_localization_error_m": mean_raw_error,
        "maximum_raw_localization_error_m": max(
            raw_errors,
            default=math.nan,
        ),
        "mean_corrected_localization_error_m": mean_corrected_error,
        "p95_corrected_localization_error_m": percentile(
            corrected_errors,
            0.95,
        ),
        "localization_improvement_percent": localization_improvement,
        "relocalization_updates": relocalizer.update_count,
        "sensor_retention_ratio": (
            total_sensor_outputs / total_sensor_inputs
            if total_sensor_inputs
            else 0.0
        ),
        "stopped_duration_s": stopped_steps * time_step_s,
        "resumed_after_obstacle": resumed_after_obstacle,
        "minimum_obstacle_distance_m": (
            minimum_obstacle_distance_m
            if config.dynamic_obstacle
            else math.nan
        ),
    }
    criteria, checks = _scenario_checks(config, metrics)
    summary: dict[str, object] = {
        "scenario": asdict(config),
        "metrics": metrics,
        "acceptance_criteria": criteria,
        "acceptance_checks": checks,
        "acceptance_passed": all(checks.values()),
    }
    return rows, summary


def evaluate_matrix(
    scenarios: list[ScenarioConfig],
) -> tuple[list[dict[str, int | float | str]], dict[str, object]]:
    """Run and aggregate a complete H5 scenario matrix. / 运行并汇总完整H5场景矩阵。"""
    if not scenarios:
        raise ValueError(
            "At least one scenario is required. / 至少需要一个场景。"
        )
    all_rows: list[dict[str, int | float | str]] = []
    summaries: list[dict[str, object]] = []
    for scenario in scenarios:
        rows, summary = run_scenario(scenario)
        all_rows.extend(rows)
        summaries.append(summary)

    shapes = sorted(
        {
            str(summary["scenario"]["track_shape"])
            for summary in summaries
        }
    )
    dynamic_summaries = [
        summary
        for summary in summaries
        if bool(summary["scenario"]["dynamic_obstacle"])
    ]
    aggregate_metrics = {
        "scenario_count": len(summaries),
        "passed_scenarios": sum(
            bool(summary["acceptance_passed"]) for summary in summaries
        ),
        "track_shapes": shapes,
        "target_laps_total": sum(
            int(summary["metrics"]["target_laps"])
            for summary in summaries
        ),
        "completed_laps_total": sum(
            int(summary["metrics"]["completed_laps"])
            for summary in summaries
        ),
        "worst_p95_cross_track_error_m": max(
            float(summary["metrics"]["p95_cross_track_error_m"])
            for summary in summaries
        ),
        "worst_p95_corrected_localization_error_m": max(
            float(
                summary["metrics"][
                    "p95_corrected_localization_error_m"
                ]
            )
            for summary in summaries
        ),
        "minimum_localization_improvement_percent": min(
            float(
                summary["metrics"][
                    "localization_improvement_percent"
                ]
            )
            for summary in summaries
        ),
        "dynamic_obstacle_scenarios": len(dynamic_summaries),
        "dynamic_obstacle_stop_and_resume_scenarios": sum(
            float(summary["metrics"]["stopped_duration_s"]) >= 0.5
            and bool(summary["metrics"]["resumed_after_obstacle"])
            for summary in dynamic_summaries
        ),
    }
    aggregate_checks = {
        "all_scenarios_passed": aggregate_metrics["passed_scenarios"]
        == aggregate_metrics["scenario_count"],
        "three_track_shapes_covered": len(shapes) == 3,
        "all_target_laps_completed": aggregate_metrics[
            "completed_laps_total"
        ]
        >= aggregate_metrics["target_laps_total"],
        "dynamic_obstacle_stop_and_resume_covered": (
            aggregate_metrics["dynamic_obstacle_scenarios"] >= 1
            and aggregate_metrics[
                "dynamic_obstacle_stop_and_resume_scenarios"
            ]
            == aggregate_metrics["dynamic_obstacle_scenarios"]
        ),
    }
    return all_rows, {
        "stage": (
            "H5 multi-lap, multi-shape, dynamic-obstacle, and "
            "robustness evaluation"
        ),
        "method": (
            "Deterministic bicycle-model regression using the project "
            "track, relocalization, pure-pursuit, sensor-fault, and "
            "safety modules"
        ),
        "scenario_summaries": summaries,
        "aggregate_metrics": aggregate_metrics,
        "aggregate_checks": aggregate_checks,
        "acceptance_passed": all(aggregate_checks.values()),
        "note": (
            "H4 separately validates the live CARLA LiDAR pipeline. "
            "This H5 matrix expands repeatable coverage before selected "
            "CARLA/ROS integration runs."
        ),
    }
