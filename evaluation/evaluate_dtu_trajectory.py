"""Evaluate DTU KISS-ICP against the recorded vehicle reference odometry.

用 DTU 数据中记录的车辆参考里程计评估 KISS-ICP 轨迹。
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import sys
from pathlib import Path

import numpy as np
from rosbags.highlevel import AnyReader


PROJECT_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_BAG = (
    PROJECT_ROOT
    / "datasets"
    / "public"
    / "dtu_roadrunners"
    / "extracted"
    / "05-2020 DTU"
    / "2020-05-26.bag"
)
DEFAULT_KISS_CSV = PROJECT_ROOT / "output" / "dtu_kiss_icp_trajectory.csv"
DEFAULT_DETAIL_CSV = PROJECT_ROOT / "output" / "dtu_trajectory_evaluation.csv"
DEFAULT_SUMMARY_JSON = PROJECT_ROOT / "output" / "dtu_trajectory_evaluation.json"
DEFAULT_PLOT_SVG = PROJECT_ROOT / "output" / "dtu_trajectory_comparison.svg"

# Keep bilingual terminal output readable on Windows. / 保证 Windows 终端能显示中英文输出。
if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
if hasattr(sys.stderr, "reconfigure"):
    sys.stderr.reconfigure(encoding="utf-8", errors="replace")


def load_kiss_trajectory(path: Path) -> tuple[np.ndarray, np.ndarray]:
    """Load timestamped KISS positions. / 读取带时间戳的 KISS 位姿。"""
    times: list[float] = []
    positions: list[list[float]] = []
    with path.open("r", encoding="utf-8", newline="") as handle:
        for row in csv.DictReader(handle):
            times.append(float(row["stamp_sec"]))
            positions.append(
                [float(row["x_m"]), float(row["y_m"]), float(row["z_m"])]
            )
    if len(times) < 10:
        raise ValueError(f"KISS trajectory has too few samples / KISS 样本不足: {len(times)}")
    return np.asarray(times), np.asarray(positions)


def load_vehicle_reference(
    path: Path,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """Read x, y, yaw, speed and distance from the DTU bag. / 读取 DTU 车辆参考量。"""
    times: list[float] = []
    positions: list[list[float]] = []
    yaws: list[float] = []
    speeds: list[float] = []
    distances: list[float] = []
    with AnyReader([path]) as reader:
        connections = [
            connection
            for connection in reader.connections
            if connection.topic == "/car_pose_estimate"
        ]
        for connection, timestamp, rawdata in reader.messages(connections=connections):
            message = reader.deserialize(rawdata, connection.msgtype)
            values = np.asarray(message.data, dtype=float)
            if len(values) < 7:
                continue
            # Float32MultiArray has no header; use its bag record time. / 此消息没有消息头，使用 bag 记录时间。
            times.append(timestamp * 1e-9)
            positions.append([values[0], values[1]])
            yaws.append(values[3])
            speeds.append(values[4])
            distances.append(values[6])
    if len(times) < 10:
        raise ValueError(f"Reference has too few samples / 参考样本不足: {len(times)}")

    order = np.argsort(times)
    sorted_times = np.asarray(times)[order]
    unique_times, unique_indices = np.unique(sorted_times, return_index=True)
    selected = order[unique_indices]
    return (
        unique_times,
        np.asarray(positions)[selected],
        np.unwrap(np.asarray(yaws)[selected]),
        np.asarray(speeds)[selected],
        np.asarray(distances)[selected],
    )


def interpolate_reference(
    query_times: np.ndarray,
    reference_times: np.ndarray,
    reference_positions: np.ndarray,
    reference_yaws: np.ndarray,
    reference_speeds: np.ndarray,
    reference_distances: np.ndarray,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """Match the two trajectories by timestamp. / 按时间戳匹配两条轨迹。"""
    valid = (query_times >= reference_times[0]) & (query_times <= reference_times[-1])
    times = query_times[valid]
    positions = np.column_stack(
        [
            np.interp(times, reference_times, reference_positions[:, axis])
            for axis in range(2)
        ]
    )
    yaws = np.interp(times, reference_times, reference_yaws)
    speeds = np.interp(times, reference_times, reference_speeds)
    distances = np.interp(times, reference_times, reference_distances)
    return valid, times, positions, yaws, speeds, distances


def rigid_align_2d(
    source: np.ndarray,
    target: np.ndarray,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Apply SE(2) rotation and translation without scale. / 只做二维旋转和平移，不改变比例。"""
    source_center = source.mean(axis=0)
    target_center = target.mean(axis=0)
    source_zero = source - source_center
    target_zero = target - target_center
    u_matrix, _, vt_matrix = np.linalg.svd(source_zero.T @ target_zero)
    rotation = vt_matrix.T @ u_matrix.T
    if np.linalg.det(rotation) < 0:
        vt_matrix[-1, :] *= -1
        rotation = vt_matrix.T @ u_matrix.T
    translation = target_center - rotation @ source_center
    aligned = (rotation @ source.T).T + translation
    return aligned, rotation, translation


def one_second_relative_errors(
    times: np.ndarray,
    estimate: np.ndarray,
    reference: np.ndarray,
) -> np.ndarray:
    """Measure one-second relative translation errors. / 计算一秒相对平移误差。"""
    target_indices = np.searchsorted(times, times + 1.0)
    source_indices = np.arange(len(times))
    valid = target_indices < len(times)
    source_indices = source_indices[valid]
    target_indices = target_indices[valid]
    time_gaps = times[target_indices] - times[source_indices]
    close_to_one_second = (time_gaps >= 0.8) & (time_gaps <= 1.2)
    source_indices = source_indices[close_to_one_second]
    target_indices = target_indices[close_to_one_second]
    estimate_delta = estimate[target_indices] - estimate[source_indices]
    reference_delta = reference[target_indices] - reference[source_indices]
    return np.linalg.norm(estimate_delta - reference_delta, axis=1)


def local_segment_metrics(
    times: np.ndarray,
    estimate: np.ndarray,
    reference: np.ndarray,
    window_seconds: float = 20.0,
) -> list[dict[str, float | int]]:
    """Measure locally aligned fixed-duration segments. / 评估局部对齐的固定时长分段。"""
    metrics: list[dict[str, float | int]] = []
    relative_times = times - times[0]
    for start in np.arange(0.0, relative_times[-1], window_seconds):
        end = min(start + window_seconds, float(relative_times[-1]))
        if end >= relative_times[-1]:
            selected = (relative_times >= start) & (relative_times <= end)
        else:
            selected = (relative_times >= start) & (relative_times < end)
        if int(selected.sum()) < 10:
            continue
        aligned, _, _ = rigid_align_2d(estimate[selected], reference[selected])
        errors = np.linalg.norm(aligned - reference[selected], axis=1)
        metrics.append(
            {
                "segment_start_s": float(start),
                "segment_end_s": float(end),
                "segment_samples": int(selected.sum()),
                "local_ate_rmse_m": float(np.sqrt(np.mean(errors**2))),
                "local_ate_p95_m": float(np.quantile(errors, 0.95)),
            }
        )
    return metrics


def path_length(points: np.ndarray) -> float:
    """Sum consecutive planar movements. / 累加连续二维位移。"""
    return float(np.linalg.norm(np.diff(points, axis=0), axis=1).sum())


def save_trajectory_svg(
    path: Path,
    estimate: np.ndarray,
    reference: np.ndarray,
    ate_rmse: float,
) -> None:
    """Create a dependency-free bilingual trajectory figure. / 生成无需额外依赖的双语轨迹图。"""
    width = 1000
    height = 720
    margin = 90
    combined = np.vstack([estimate, reference])
    minimum = combined.min(axis=0)
    maximum = combined.max(axis=0)
    span = np.maximum(maximum - minimum, 1e-6)
    scale = min((width - 2 * margin) / span[0], (height - 2 * margin) / span[1])

    # Keep the same scale on both axes. / 两个坐标轴使用相同比例。
    def project(points: np.ndarray) -> np.ndarray:
        x_values = margin + (points[:, 0] - minimum[0]) * scale
        y_values = height - margin - (points[:, 1] - minimum[1]) * scale
        return np.column_stack([x_values, y_values])

    estimate_pixels = project(estimate)
    reference_pixels = project(reference)

    def polyline(points: np.ndarray) -> str:
        return " ".join(f"{x_value:.1f},{y_value:.1f}" for x_value, y_value in points)

    error_lines: list[str] = []
    for index in range(0, len(estimate_pixels), 40):
        ex, ey = estimate_pixels[index]
        rx, ry = reference_pixels[index]
        error_lines.append(
            f'<line x1="{ex:.1f}" y1="{ey:.1f}" x2="{rx:.1f}" y2="{ry:.1f}" '
            'stroke="#cbd5e1" stroke-width="1" opacity="0.65" />'
        )

    reference_start = reference_pixels[0]
    reference_end = reference_pixels[-1]
    estimate_end = estimate_pixels[-1]
    svg = f"""<?xml version="1.0" encoding="UTF-8"?>
<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">
  <rect width="100%" height="100%" fill="#ffffff" />
  <text x="{width / 2}" y="38" text-anchor="middle" font-family="Arial, sans-serif" font-size="24" font-weight="700" fill="#0f172a">DTU trajectory comparison / DTU轨迹比较</text>
  <text x="{width / 2}" y="65" text-anchor="middle" font-family="Arial, sans-serif" font-size="15" fill="#475569">SE(2) aligned, no scale change / 二维刚体对齐，不改变比例 · ATE RMSE {ate_rmse:.3f} m</text>
  <rect x="{margin}" y="{margin}" width="{width - 2 * margin}" height="{height - 2 * margin}" fill="#f8fafc" stroke="#94a3b8" />
  {''.join(error_lines)}
  <polyline points="{polyline(reference_pixels)}" fill="none" stroke="#2563eb" stroke-width="4" stroke-linejoin="round" />
  <polyline points="{polyline(estimate_pixels)}" fill="none" stroke="#f97316" stroke-width="3" stroke-linejoin="round" />
  <circle cx="{reference_start[0]:.1f}" cy="{reference_start[1]:.1f}" r="8" fill="#16a34a" />
  <circle cx="{reference_end[0]:.1f}" cy="{reference_end[1]:.1f}" r="8" fill="#2563eb" />
  <circle cx="{estimate_end[0]:.1f}" cy="{estimate_end[1]:.1f}" r="8" fill="#f97316" />
  <line x1="{margin + 20}" y1="{height - 46}" x2="{margin + 70}" y2="{height - 46}" stroke="#2563eb" stroke-width="4" />
  <text x="{margin + 80}" y="{height - 40}" font-family="Arial, sans-serif" font-size="15" fill="#0f172a">Vehicle odometry reference / 车辆里程计参考</text>
  <line x1="{margin + 410}" y1="{height - 46}" x2="{margin + 460}" y2="{height - 46}" stroke="#f97316" stroke-width="4" />
  <text x="{margin + 470}" y="{height - 40}" font-family="Arial, sans-serif" font-size="15" fill="#0f172a">KISS-ICP estimate / KISS-ICP估计</text>
  <text x="{width / 2}" y="{height - 12}" text-anchor="middle" font-family="Arial, sans-serif" font-size="13" fill="#64748b">Grey links show sampled position differences / 灰线表示抽样位置差</text>
</svg>
"""
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(svg, encoding="utf-8")


def save_results(
    detail_path: Path,
    summary_path: Path,
    times: np.ndarray,
    kiss_raw: np.ndarray,
    kiss_aligned: np.ndarray,
    reference: np.ndarray,
    reference_yaws: np.ndarray,
    reference_speeds: np.ndarray,
    reference_distances: np.ndarray,
    rotation: np.ndarray,
    translation: np.ndarray,
) -> dict[str, object]:
    """Save detailed errors and a report summary. / 保存逐点误差和报告汇总。"""
    errors = np.linalg.norm(kiss_aligned - reference, axis=1)
    relative_errors = one_second_relative_errors(times, kiss_aligned, reference)
    segment_metrics = local_segment_metrics(times, kiss_raw[:, :2], reference)
    worst_segment = max(segment_metrics, key=lambda item: item["local_ate_rmse_m"])
    kiss_length = path_length(kiss_aligned)
    driven_distance = float(reference_distances[-1] - reference_distances[0])
    length_difference_percent = (
        100.0 * (kiss_length - driven_distance) / driven_distance
        if driven_distance > 0.0
        else math.nan
    )

    detail_path.parent.mkdir(parents=True, exist_ok=True)
    with detail_path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(
            [
                "stamp_sec",
                "kiss_x_m",
                "kiss_y_m",
                "kiss_z_m",
                "aligned_x_m",
                "aligned_y_m",
                "reference_x_m",
                "reference_y_m",
                "reference_yaw_rad",
                "reference_speed_mps",
                "reference_driven_distance_m",
                "position_difference_m",
            ]
        )
        for index in range(len(times)):
            writer.writerow(
                [
                    times[index],
                    *kiss_raw[index],
                    *kiss_aligned[index],
                    *reference[index],
                    reference_yaws[index],
                    reference_speeds[index],
                    reference_distances[index],
                    errors[index],
                ]
            )

    summary: dict[str, object] = {
        "dataset": "DTU Roadrunners Autonomous Ecocar Race Track Dataset",
        "sequence": "05-2020 DTU/2020-05-26.bag",
        "sensor": "Velodyne VLP-16",
        "comparison": "KISS-ICP versus recorded vehicle self-odometry",
        "reference_quality": "vehicle odometry reference; not RTK or survey ground truth",
        "license": "CC BY-NC-SA 4.0",
        "matched_samples": int(len(times)),
        "matched_duration_seconds": float(times[-1] - times[0]),
        "reference_driven_distance_m": driven_distance,
        "kiss_aligned_path_length_m": kiss_length,
        "path_length_difference_percent": float(length_difference_percent),
        "ate_mean_m": float(errors.mean()),
        "ate_median_m": float(np.median(errors)),
        "ate_rmse_m": float(np.sqrt(np.mean(errors**2))),
        "ate_p95_m": float(np.quantile(errors, 0.95)),
        "ate_max_m": float(errors.max()),
        "final_position_difference_m": float(errors[-1]),
        "rpe_1s_samples": int(len(relative_errors)),
        "rpe_1s_mean_m": float(relative_errors.mean()),
        "rpe_1s_rmse_m": float(np.sqrt(np.mean(relative_errors**2))),
        "rpe_1s_p95_m": float(np.quantile(relative_errors, 0.95)),
        "local_20s_segments": segment_metrics,
        "worst_local_segment_start_s": worst_segment["segment_start_s"],
        "worst_local_segment_end_s": worst_segment["segment_end_s"],
        "worst_local_segment_rmse_m": worst_segment["local_ate_rmse_m"],
        "alignment_yaw_degrees": float(
            math.degrees(math.atan2(rotation[1, 0], rotation[0, 0]))
        ),
        "alignment_rotation_2d": rotation.tolist(),
        "alignment_translation_m": translation.tolist(),
    }
    with summary_path.open("w", encoding="utf-8") as handle:
        json.dump(summary, handle, ensure_ascii=False, indent=2)
        handle.write("\n")
    return summary


def parse_args() -> argparse.Namespace:
    """Read optional file paths. / 读取可选文件路径。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bag", type=Path, default=DEFAULT_BAG)
    parser.add_argument("--kiss-csv", type=Path, default=DEFAULT_KISS_CSV)
    parser.add_argument("--detail-csv", type=Path, default=DEFAULT_DETAIL_CSV)
    parser.add_argument("--summary-json", type=Path, default=DEFAULT_SUMMARY_JSON)
    parser.add_argument("--plot-svg", type=Path, default=DEFAULT_PLOT_SVG)
    return parser.parse_args()


def main() -> None:
    """Run the complete DTU trajectory evaluation. / 运行完整 DTU 轨迹评估。"""
    args = parse_args()
    kiss_times, kiss_positions = load_kiss_trajectory(args.kiss_csv)
    reference = load_vehicle_reference(args.bag)
    valid, times, reference_positions, yaws, speeds, distances = interpolate_reference(
        kiss_times, *reference
    )
    kiss_matched = kiss_positions[valid]
    if len(times) < 50:
        raise ValueError(f"Too few matched samples / 匹配样本不足: {len(times)}")

    aligned, rotation, translation = rigid_align_2d(
        kiss_matched[:, :2], reference_positions
    )
    summary = save_results(
        args.detail_csv,
        args.summary_json,
        times,
        kiss_matched,
        aligned,
        reference_positions,
        yaws,
        speeds,
        distances,
        rotation,
        translation,
    )
    save_trajectory_svg(
        args.plot_svg,
        aligned,
        reference_positions,
        float(summary["ate_rmse_m"]),
    )

    # Refresh presentation copies without changing machine files. / 刷新展示副本，不修改机器文件。
    from export_bilingual_reports import export_all

    export_all(PROJECT_ROOT / "output", PROJECT_ROOT / "output" / "bilingual")

    print(f"Matched samples / 匹配样本：{summary['matched_samples']}")
    print(f"Matched duration / 匹配时长：{summary['matched_duration_seconds']:.1f} s")
    print(f"Driven distance / 行驶距离：{summary['reference_driven_distance_m']:.1f} m")
    print(f"ATE RMSE / 绝对轨迹误差 RMSE：{summary['ate_rmse_m']:.3f} m")
    print(f"RPE 1 s RMSE / 一秒相对误差 RMSE：{summary['rpe_1s_rmse_m']:.3f} m")
    print(
        "Worst local segment / 最差局部分段："
        f"{summary['worst_local_segment_start_s']:.0f}–"
        f"{summary['worst_local_segment_end_s']:.0f} s, "
        f"RMSE {summary['worst_local_segment_rmse_m']:.3f} m"
    )
    print("Note / 注意：参考是车辆自身里程计，不是 RTK 或测量真值。")
    print(f"Summary / 汇总：{args.summary_json}")
    print(f"Trajectory figure / 轨迹图：{args.plot_svg}")
    print("DTU_TRAJECTORY_EVALUATION_OK / DTU 轨迹评估成功")


if __name__ == "__main__":
    main()
