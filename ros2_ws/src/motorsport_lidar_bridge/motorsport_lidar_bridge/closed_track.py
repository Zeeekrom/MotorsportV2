"""General closed-track geometry and lap tracking. / 通用闭环赛道几何与圈数跟踪。"""

from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np


SUPPORTED_TRACK_SHAPES = ("circle", "oval", "technical")
DEFAULT_GATE_COUNTS = {
    "circle": 24,
    "oval": 32,
    "technical": 36,
}


@dataclass(frozen=True)
class TrackGeometry:
    """Sampled centreline and cone boundaries. / 离散中心线与锥桶边界。"""

    shape: str
    centreline: np.ndarray
    tangents: np.ndarray
    left_cones: np.ndarray
    right_cones: np.ndarray
    segment_lengths_m: np.ndarray
    cumulative_lengths_m: np.ndarray
    length_m: float
    half_width_m: float


@dataclass(frozen=True)
class TrackProjection:
    """Nearest centreline projection for one position. / 单个位置的最近中心线投影。"""

    segment_index: int
    segment_fraction: float
    progress_m: float
    projected_position: np.ndarray
    tangent: np.ndarray
    cross_track_error_m: float
    signed_cross_track_error_m: float


@dataclass(frozen=True)
class TrackTruth:
    """Ground-truth progress and control errors. / 真值进度与控制误差。"""

    progress_m: float
    lap_progress_percent: float
    completed_laps: int
    cross_track_error_m: float
    heading_error_degrees: float
    distance_to_start_m: float


def wrap_degrees(angle_degrees: float) -> float:
    """Wrap one angle to [-180, 180). / 把角度归一化到[-180, 180)。"""
    return (float(angle_degrees) + 180.0) % 360.0 - 180.0


def _shape_samples(
    shape: str,
    angles: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    """Return centre points and analytic derivatives. / 返回中心点及解析导数。"""
    if shape == "circle":
        radius = 10.0
        x_position = radius * np.sin(angles)
        y_position = radius * (1.0 - np.cos(angles))
        x_derivative = radius * np.cos(angles)
        y_derivative = radius * np.sin(angles)
    elif shape == "oval":
        x_radius = 14.0
        y_radius = 8.0
        x_position = x_radius * np.sin(angles)
        y_position = y_radius * (1.0 - np.cos(angles))
        x_derivative = x_radius * np.cos(angles)
        y_derivative = y_radius * np.sin(angles)
    elif shape == "technical":
        # The second harmonic creates unequal left/right sectors without
        # self-intersection. / 二次谐波形成不对称弯道，同时避免自相交。
        x_position = 11.0 * np.sin(angles) + 2.0 * np.sin(2.0 * angles)
        y_position = (
            9.0 * (1.0 - np.cos(angles))
            + 1.5 * np.sin(angles) ** 2
        )
        x_derivative = (
            11.0 * np.cos(angles) + 4.0 * np.cos(2.0 * angles)
        )
        y_derivative = (
            9.0 * np.sin(angles)
            + 3.0 * np.sin(angles) * np.cos(angles)
        )
    else:
        supported = ", ".join(SUPPORTED_TRACK_SHAPES)
        raise ValueError(
            f"Unsupported track shape: {shape}; expected {supported}. / "
            f"不支持的赛道形状：{shape}；可选值为{supported}。"
        )

    centreline = np.column_stack((x_position, y_position))
    derivatives = np.column_stack((x_derivative, y_derivative))
    return centreline, derivatives


def generate_closed_track(
    shape: str,
    *,
    gate_count: int | None = None,
    half_width_m: float = 2.2,
) -> TrackGeometry:
    """Generate one deterministic closed cone track. / 生成一条确定性闭环锥桶赛道。"""
    normalized_shape = str(shape).strip().lower()
    if normalized_shape not in SUPPORTED_TRACK_SHAPES:
        supported = ", ".join(SUPPORTED_TRACK_SHAPES)
        raise ValueError(
            f"Unsupported track shape: {shape}; expected {supported}. / "
            f"不支持的赛道形状：{shape}；可选值为{supported}。"
        )
    resolved_gate_count = (
        DEFAULT_GATE_COUNTS[normalized_shape]
        if gate_count is None
        else int(gate_count)
    )
    if resolved_gate_count < 12:
        raise ValueError(
            "gate_count must be at least 12. / gate_count必须至少为12。"
        )
    if half_width_m <= 0.0:
        raise ValueError(
            "half_width_m must be positive. / half_width_m必须为正数。"
        )

    # Keep the first gate at (0, 0) with heading +X for every shape.
    # 所有形状的第一个门均位于(0, 0)，起始航向为+X。
    angles = np.linspace(
        0.0,
        2.0 * math.pi,
        resolved_gate_count,
        endpoint=False,
        dtype=np.float64,
    )
    centreline, derivatives = _shape_samples(normalized_shape, angles)
    derivative_norms = np.linalg.norm(derivatives, axis=1, keepdims=True)
    tangents = derivatives / np.maximum(derivative_norms, 1e-9)
    left_normals = np.column_stack((-tangents[:, 1], tangents[:, 0]))
    left_cones = centreline + float(half_width_m) * left_normals
    right_cones = centreline - float(half_width_m) * left_normals

    # Include the closing segment in total and cumulative length.
    # 总长度和累计长度均包含闭合赛段。
    segment_vectors = np.roll(centreline, -1, axis=0) - centreline
    segment_lengths = np.linalg.norm(segment_vectors, axis=1)
    cumulative_lengths = np.concatenate(
        (np.asarray([0.0]), np.cumsum(segment_lengths))
    )
    return TrackGeometry(
        shape=normalized_shape,
        centreline=centreline.astype(np.float32),
        tangents=tangents.astype(np.float32),
        left_cones=left_cones.astype(np.float32),
        right_cones=right_cones.astype(np.float32),
        segment_lengths_m=segment_lengths.astype(np.float64),
        cumulative_lengths_m=cumulative_lengths.astype(np.float64),
        length_m=float(cumulative_lengths[-1]),
        half_width_m=float(half_width_m),
    )


def project_to_track(
    track: TrackGeometry,
    position_xy: np.ndarray,
) -> TrackProjection:
    """Project a position onto the nearest closed segment. / 把位置投影到最近闭合赛段。"""
    position = np.asarray(position_xy, dtype=np.float64).reshape(-1)[:2]
    starts = track.centreline.astype(np.float64)
    ends = np.roll(starts, -1, axis=0)
    vectors = ends - starts
    squared_lengths = np.sum(vectors * vectors, axis=1)
    fractions = np.sum((position - starts) * vectors, axis=1) / np.maximum(
        squared_lengths,
        1e-12,
    )
    fractions = np.clip(fractions, 0.0, 1.0)
    projections = starts + fractions[:, None] * vectors
    offsets = position - projections
    distances = np.linalg.norm(offsets, axis=1)
    segment_index = int(np.argmin(distances))
    segment_length = float(track.segment_lengths_m[segment_index])
    tangent = vectors[segment_index] / max(segment_length, 1e-9)
    left_normal = np.asarray([-tangent[1], tangent[0]])
    signed_error = float(offsets[segment_index] @ left_normal)
    progress = float(
        track.cumulative_lengths_m[segment_index]
        + fractions[segment_index] * segment_length
    )
    return TrackProjection(
        segment_index=segment_index,
        segment_fraction=float(fractions[segment_index]),
        progress_m=progress,
        projected_position=projections[segment_index].astype(np.float32),
        tangent=tangent.astype(np.float32),
        cross_track_error_m=float(distances[segment_index]),
        signed_cross_track_error_m=signed_error,
    )


def sample_at_progress(
    track: TrackGeometry,
    progress_m: float,
) -> tuple[np.ndarray, np.ndarray]:
    """Interpolate position and tangent at wrapped progress. / 在循环进度处插值位置与切向。"""
    wrapped_progress = float(progress_m) % track.length_m
    segment_index = int(
        np.searchsorted(
            track.cumulative_lengths_m,
            wrapped_progress,
            side="right",
        )
        - 1
    )
    segment_index = min(segment_index, len(track.centreline) - 1)
    segment_start = track.centreline[segment_index].astype(np.float64)
    segment_end = track.centreline[
        (segment_index + 1) % len(track.centreline)
    ].astype(np.float64)
    segment_length = float(track.segment_lengths_m[segment_index])
    fraction = (
        wrapped_progress - track.cumulative_lengths_m[segment_index]
    ) / max(segment_length, 1e-9)
    position = segment_start + fraction * (segment_end - segment_start)
    tangent = (segment_end - segment_start) / max(segment_length, 1e-9)
    return position.astype(np.float32), tangent.astype(np.float32)


def sample_path_ahead(
    track: TrackGeometry,
    progress_m: float,
    *,
    horizon_m: float = 24.0,
    spacing_m: float = 2.0,
) -> np.ndarray:
    """Sample ordered centre points ahead of the vehicle. / 采样车辆前方的有序中心点。"""
    if horizon_m <= 0.0 or spacing_m <= 0.0:
        raise ValueError(
            "horizon_m and spacing_m must be positive. / "
            "horizon_m和spacing_m必须为正数。"
        )
    offsets = np.arange(
        spacing_m,
        horizon_m + 0.5 * spacing_m,
        spacing_m,
        dtype=np.float64,
    )
    positions = [
        sample_at_progress(track, progress_m + float(offset))[0]
        for offset in offsets
    ]
    return np.asarray(positions, dtype=np.float32)


def sample_ros_path_ahead(
    track: TrackGeometry,
    ros_position_xy: np.ndarray,
    *,
    horizon_m: float = 24.0,
    spacing_m: float = 2.0,
) -> np.ndarray:
    """Sample a saved CARLA-local route in the reflected ROS map frame.

    在经过Y轴反射的ROS地图坐标系中采样已保存的CARLA局部路线。
    """
    carla_position = np.asarray(
        ros_position_xy,
        dtype=np.float32,
    ).reshape(-1)[:2].copy()
    carla_position[1] *= -1.0
    projection = project_to_track(track, carla_position)
    ros_path = sample_path_ahead(
        track,
        projection.progress_m,
        horizon_m=horizon_m,
        spacing_m=spacing_m,
    )
    ros_path[:, 1] *= -1.0
    return ros_path


class PolylineLapTracker:
    """Track continuous progress on any sampled closed course. / 跟踪任意离散闭环赛道的连续进度。"""

    def __init__(self, track: TrackGeometry) -> None:
        self.track = track
        self.last_wrapped_progress_m: float | None = None
        self.unwrapped_progress_m = 0.0

    def update(
        self,
        position_xy: np.ndarray,
        heading_degrees: float,
    ) -> TrackTruth:
        """Update lap count and truth errors. / 更新圈数和真值误差。"""
        position = np.asarray(position_xy, dtype=np.float64).reshape(-1)[:2]
        projection = project_to_track(self.track, position)
        wrapped_progress = projection.progress_m
        if self.last_wrapped_progress_m is None:
            self.unwrapped_progress_m = wrapped_progress
        else:
            delta = wrapped_progress - self.last_wrapped_progress_m
            half_length = 0.5 * self.track.length_m
            if delta > half_length:
                delta -= self.track.length_m
            elif delta < -half_length:
                delta += self.track.length_m
            self.unwrapped_progress_m = max(
                0.0,
                self.unwrapped_progress_m + delta,
            )
        self.last_wrapped_progress_m = wrapped_progress

        reference_heading = math.degrees(
            math.atan2(
                float(projection.tangent[1]),
                float(projection.tangent[0]),
            )
        )
        completed_laps = int(
            self.unwrapped_progress_m // self.track.length_m
        )
        return TrackTruth(
            progress_m=self.unwrapped_progress_m,
            lap_progress_percent=(
                100.0 * self.unwrapped_progress_m / self.track.length_m
            ),
            completed_laps=completed_laps,
            cross_track_error_m=projection.cross_track_error_m,
            heading_error_degrees=wrap_degrees(
                float(heading_degrees) - reference_heading
            ),
            distance_to_start_m=float(
                np.linalg.norm(position - self.track.centreline[0, :2])
            ),
        )
