"""Build a centre path from paired left/right cones. / 通过左右锥桶配对生成中心路径。"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class CenterPath:
    """Pure path-planning result. / 不依赖ROS消息的路径规划结果。"""

    points: np.ndarray
    widths: np.ndarray
    pair_indices: np.ndarray


class ConeMapAccumulator:
    """Merge repeated stable cones into a bounded local map. / 将重复稳定锥桶合并为有界局部地图。"""

    def __init__(
        self,
        *,
        merge_distance: float = 0.75,
        maximum_points: int = 500,
    ) -> None:
        self.merge_distance = float(merge_distance)
        self.maximum_points = int(maximum_points)
        self._points: np.ndarray | None = None
        self._counts = np.empty(0, dtype=np.int32)

    def update(
        self,
        new_points: np.ndarray,
        *,
        allow_new: bool = True,
    ) -> None:
        """Merge observations and optionally lock the landmark set.

        合并观测，并可选择锁定现有地标集合。
        """
        points = np.asarray(new_points, dtype=np.float32)
        if points.size == 0:
            return
        if points.ndim != 2 or points.shape[1] < 2:
            raise ValueError(
                "new_points must have shape (N, 2+) / new_points必须是(N, 2+)形状"
            )
        points = points[np.isfinite(points).all(axis=1)]
        if not len(points):
            return
        if self._points is None:
            self._points = np.empty((0, points.shape[1]), dtype=np.float32)
        if points.shape[1] != self._points.shape[1]:
            raise ValueError(
                "Point dimensions changed / 锥桶地图点的维度发生变化"
            )

        for point in points:
            if len(self._points):
                distances = np.linalg.norm(self._points[:, :2] - point[:2], axis=1)
                nearest = int(np.argmin(distances))
            else:
                distances = np.empty(0, dtype=np.float32)
                nearest = -1

            if nearest >= 0 and distances[nearest] <= self.merge_distance:
                count = int(self._counts[nearest])
                self._points[nearest] = (
                    self._points[nearest] * count + point
                ) / (count + 1)
                self._counts[nearest] = count + 1
            elif allow_new:
                self._points = np.vstack((self._points, point.reshape(1, -1)))
                self._counts = np.append(self._counts, 1)

        # Keep the most frequently observed landmarks if the map reaches its cap.
        # 地图达到上限时保留观测次数最多的地标。
        if len(self._points) > self.maximum_points:
            selected = np.argsort(self._counts)[-self.maximum_points :]
            self._points = self._points[selected]
            self._counts = self._counts[selected]

    def prune(self, centre: np.ndarray, maximum_distance: float = 60.0) -> None:
        """Remove landmarks outside the local-map radius. / 删除局部地图半径外的地标。"""
        if self._points is None or not len(self._points):
            return
        centre_xy = np.asarray(centre, dtype=np.float32).reshape(-1)[:2]
        keep = (
            np.linalg.norm(self._points[:, :2] - centre_xy, axis=1)
            <= maximum_distance
        )
        self._points = self._points[keep]
        self._counts = self._counts[keep]

    def points(self, minimum_observations: int = 1) -> np.ndarray:
        """Return confirmed landmarks without exposing internal state. / 返回已确认地标且不暴露内部状态。"""
        if self._points is None:
            return np.empty((0, 3), dtype=np.float32)
        selected = self._counts >= int(minimum_observations)
        return self._points[selected].copy()

    @property
    def size(self) -> int:
        """Return the number of accumulated landmarks. / 返回累计地标数量。"""
        return 0 if self._points is None else len(self._points)


def empty_center_path() -> CenterPath:
    """Return a shape-stable empty result. / 返回形状固定的空结果。"""
    return CenterPath(
        points=np.empty((0, 2), dtype=np.float32),
        widths=np.empty(0, dtype=np.float32),
        pair_indices=np.empty((0, 2), dtype=np.int32),
    )


def plan_center_path(
    cone_points: np.ndarray,
    *,
    minimum_pairs: int = 2,
    maximum_longitudinal_gap: float = 2.5,
    minimum_track_width: float = 2.0,
    maximum_track_width: float = 8.0,
    maximum_pair_step: float = 12.0,
    vehicle_position: np.ndarray | tuple[float, float] = (0.0, 0.0),
    forward_direction: np.ndarray | tuple[float, float] = (1.0, 0.0),
    minimum_forward_distance: float = -2.0,
    maximum_forward_distance: float = 35.0,
) -> CenterPath:
    """Pair cones in a vehicle-aligned frame and return midpoints. / 在车辆对齐坐标中配对锥桶并返回中点。"""
    points = np.asarray(cone_points, dtype=np.float32)
    if points.size == 0:
        return empty_center_path()
    if points.ndim != 2 or points.shape[1] < 2:
        raise ValueError(
            "cone_points must have shape (N, 2+) / cone_points必须是(N, 2+)形状"
        )

    # Keep finite planar coordinates only. / 只保留有限的平面坐标。
    planar = points[:, :2]
    source_indices = np.arange(len(planar), dtype=np.int32)
    finite = np.isfinite(planar).all(axis=1)
    planar = planar[finite]
    source_indices = source_indices[finite]

    # Build forward/left axes from the current vehicle heading.
    # 根据当前车辆朝向建立前向/左向坐标轴。
    vehicle_xy = np.asarray(vehicle_position, dtype=np.float32).reshape(-1)[:2]
    forward = np.asarray(forward_direction, dtype=np.float32).reshape(-1)[:2]
    forward_norm = float(np.linalg.norm(forward))
    if forward_norm < 1e-6:
        raise ValueError(
            "forward_direction must be non-zero / forward_direction不能是零向量"
        )
    forward /= forward_norm
    relative = planar - vehicle_xy
    longitudinal = relative @ forward
    in_range = (
        (longitudinal >= minimum_forward_distance)
        & (longitudinal <= maximum_forward_distance)
    )
    planar = planar[in_range]
    source_indices = source_indices[in_range]
    if len(planar) < 2:
        return empty_center_path()

    # Select the nearest cross-track pair, move to its midpoint, and update
    # the local forward direction. This follows a curve instead of using one
    # fixed left/right split for the entire visible corridor.
    # 选择最近的横向锥桶对，移动到其中点并更新局部前进方向，从而沿弯道逐段推进。
    available = set(range(len(planar)))
    current_position = vehicle_xy.copy()
    current_forward = forward.copy()
    accepted: list[tuple[float, np.ndarray, float, int, int]] = []
    travelled = 0.0
    while len(available) >= 2:
        candidate_indices = sorted(available)
        best: tuple[
            float,
            int,
            int,
            np.ndarray,
            float,
            float,
        ] | None = None
        for first_offset, first_index in enumerate(candidate_indices[:-1]):
            for second_index in candidate_indices[first_offset + 1 :]:
                pair_vector = planar[second_index] - planar[first_index]
                width = float(np.linalg.norm(pair_vector))
                if not minimum_track_width <= width <= maximum_track_width:
                    continue

                # A valid gate pair is mostly perpendicular to the current path.
                # 合格锥桶对相对当前路径方向应主要呈横向分布。
                along_pair = abs(float(pair_vector @ current_forward))
                if along_pair > maximum_longitudinal_gap:
                    continue

                midpoint = 0.5 * (
                    planar[first_index] + planar[second_index]
                )
                midpoint_delta = midpoint - current_position
                step_forward = float(midpoint_delta @ current_forward)
                if not 0.25 <= step_forward <= maximum_pair_step:
                    continue
                step_distance = float(np.linalg.norm(midpoint_delta))
                if step_distance > maximum_pair_step * 1.25:
                    continue

                # Prefer the nearest forward pair and the most transverse gate.
                # 优先选择最近的前向锥桶对及横向程度最高的门。
                score = step_forward + 0.25 * along_pair
                if best is None or score < best[0]:
                    best = (
                        score,
                        first_index,
                        second_index,
                        midpoint,
                        width,
                        step_distance,
                    )

        if best is None:
            break
        _, first_index, second_index, midpoint, width, step_distance = best
        step_direction = midpoint - current_position
        step_direction /= max(float(np.linalg.norm(step_direction)), 1e-6)

        # Label the positive-left endpoint first for stable reporting.
        # 报告时把当前局部坐标中位于左侧的端点放在前面。
        local_left = np.asarray(
            [-current_forward[1], current_forward[0]],
            dtype=np.float32,
        )
        first_lateral = float(
            (planar[first_index] - midpoint) @ local_left
        )
        if first_lateral >= 0.0:
            left_index, right_index = first_index, second_index
        else:
            left_index, right_index = second_index, first_index

        travelled += step_distance
        accepted.append(
            (
                travelled,
                midpoint,
                width,
                int(source_indices[left_index]),
                int(source_indices[right_index]),
            )
        )
        available.remove(first_index)
        available.remove(second_index)
        current_position = midpoint
        # Blend directions to avoid reacting sharply to noisy midpoints.
        # 混合新旧方向，避免对有噪声的中点产生突然转向。
        current_forward = 0.25 * current_forward + 0.75 * step_direction
        current_forward /= max(float(np.linalg.norm(current_forward)), 1e-6)

    if len(accepted) < minimum_pairs:
        return empty_center_path()

    # Publish in forward order so controllers can follow the path directly.
    # 按前进方向排序，使控制器可以直接跟随。
    accepted.sort(key=lambda item: item[0])
    return CenterPath(
        points=np.asarray([item[1] for item in accepted], dtype=np.float32),
        widths=np.asarray([item[2] for item in accepted], dtype=np.float32),
        pair_indices=np.asarray(
            [[item[3], item[4]] for item in accepted],
            dtype=np.int32,
        ),
    )
