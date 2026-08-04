"""Evaluate the DTU RRC KISS-ICP trajectory against independent GPS/RTK.

使用独立 GPS/RTK 位置评估 DTU RRC 的 KISS-ICP 轨迹。
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import sys
from collections import Counter
from datetime import datetime, timedelta, timezone
from pathlib import Path

import numpy as np

# Allow both direct execution and module-style tests. / 同时支持直接运行与模块测试。
TOOLS_DIR = Path(__file__).resolve().parent
if str(TOOLS_DIR) not in sys.path:
    sys.path.insert(0, str(TOOLS_DIR))

from evaluate_dtu_trajectory import (
    load_kiss_trajectory,
    one_second_relative_errors,
    path_length,
    rigid_align_2d,
    save_trajectory_svg,
)


PROJECT_ROOT = Path(__file__).resolve().parents[1]
GPS_ROOT = (
    PROJECT_ROOT
    / "datasets"
    / "public"
    / "dtu_roadrunners"
    / "rrc_subset"
    / "03-2020 RRC"
)
DEFAULT_POSITION_CSV = GPS_ROOT / "GPS_2020-03-11-15-39_position_log.csv"
DEFAULT_VELOCITY_CSV = GPS_ROOT / "GPS_2020-03-11-15-39_velocity_log.csv"
DEFAULT_KISS_CSV = PROJECT_ROOT / "output" / "dtu_rtk_kiss_icp_trajectory.csv"
DEFAULT_DETAIL_CSV = PROJECT_ROOT / "output" / "dtu_rtk_trajectory_evaluation.csv"
DEFAULT_SUMMARY_JSON = PROJECT_ROOT / "output" / "dtu_rtk_trajectory_evaluation.json"
DEFAULT_FULL_SVG = PROJECT_ROOT / "output" / "dtu_rtk_full_gps_comparison.svg"
DEFAULT_FIXED_SVG = PROJECT_ROOT / "output" / "dtu_rtk_fixed_comparison.svg"

# Copenhagen was UTC+1 on 11 March 2020, before daylight-saving time. / 2020年3月11日哥本哈根为UTC+1。
RECORDING_TIMEZONE = timezone(timedelta(hours=1))
# GPS time was 18 seconds ahead of UTC in March 2020. / 2020年3月GPS时间比UTC快18秒。
GPS_MINUS_UTC_SECONDS = 18.0
GPS_FLAG_NAMES = {
    0: "Invalid / 无效",
    1: "Single Point Position / 单点定位",
    2: "Differential GNSS / 差分GNSS",
    3: "Float RTK / 浮点RTK",
    4: "Fixed RTK / 固定RTK",
    5: "Undefined / 未定义",
    6: "SBAS Position / SBAS定位",
}

if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
if hasattr(sys.stderr, "reconfigure"):
    sys.stderr.reconfigure(encoding="utf-8", errors="replace")


def read_csv(path: Path) -> list[dict[str, str]]:
    """Read one UTF-8 GPS log. / 读取一个 UTF-8 GPS 日志。"""
    with path.open("r", encoding="utf-8-sig", newline="") as handle:
        return list(csv.DictReader(handle))


def geodetic_to_ecef(
    latitude_degrees: np.ndarray,
    longitude_degrees: np.ndarray,
    altitude_m: np.ndarray,
) -> np.ndarray:
    """Convert WGS-84 latitude/longitude/altitude to ECEF. / 把WGS-84经纬高转换为ECEF。"""
    semi_major_axis = 6_378_137.0
    eccentricity_squared = 6.69437999014e-3
    latitude = np.radians(latitude_degrees)
    longitude = np.radians(longitude_degrees)
    prime_vertical = semi_major_axis / np.sqrt(
        1.0 - eccentricity_squared * np.sin(latitude) ** 2
    )
    x_value = (prime_vertical + altitude_m) * np.cos(latitude) * np.cos(longitude)
    y_value = (prime_vertical + altitude_m) * np.cos(latitude) * np.sin(longitude)
    z_value = (
        prime_vertical * (1.0 - eccentricity_squared) + altitude_m
    ) * np.sin(latitude)
    return np.column_stack([x_value, y_value, z_value])


def ecef_to_local_enu(
    ecef: np.ndarray,
    origin_latitude_degrees: float,
    origin_longitude_degrees: float,
    origin_ecef: np.ndarray,
) -> np.ndarray:
    """Rotate ECEF offsets into local east/north/up. / 把ECEF偏移旋转到本地东/北/上坐标。"""
    latitude = math.radians(origin_latitude_degrees)
    longitude = math.radians(origin_longitude_degrees)
    rotation = np.asarray(
        [
            [-math.sin(longitude), math.cos(longitude), 0.0],
            [
                -math.sin(latitude) * math.cos(longitude),
                -math.sin(latitude) * math.sin(longitude),
                math.cos(latitude),
            ],
            [
                math.cos(latitude) * math.cos(longitude),
                math.cos(latitude) * math.sin(longitude),
                math.sin(latitude),
            ],
        ]
    )
    return (rotation @ (ecef - origin_ecef).T).T


def load_gps_reference(
    position_path: Path,
    velocity_path: Path,
) -> dict[str, np.ndarray]:
    """Join position to GPS measurement time by TOW. / 按GPS周内秒连接位置与GPS测量时间。"""
    position_rows = read_csv(position_path)
    velocity_rows = read_csv(velocity_path)
    velocity_by_tow = {round(float(row["tow(sec)"]), 3): row for row in velocity_rows}

    times: list[float] = []
    latitudes: list[float] = []
    longitudes: list[float] = []
    altitudes: list[float] = []
    horizontal_accuracy: list[float] = []
    vertical_accuracy: list[float] = []
    satellites: list[int] = []
    flags: list[int] = []
    speeds: list[float] = []
    receive_latencies: list[float] = []
    for position in position_rows:
        tow = round(float(position["tow(sec)"]), 3)
        velocity = velocity_by_tow.get(tow)
        if velocity is None:
            continue
        # The PC receive time is about four seconds late in this recording.
        # Use GPS measurement time and convert GPS time to UTC instead.
        # 该数据的电脑接收时间约滞后4秒，因此使用GPS测量时间并转换为UTC。
        pc_time = datetime.strptime(
            velocity["pc_time"], "%Y-%m-%d %H:%M:%S.%f"
        ).replace(tzinfo=RECORDING_TIMEZONE)
        gps_time = datetime.strptime(
            velocity["gps_time"], "%Y-%m-%d %H:%M:%S.%f"
        ).replace(tzinfo=timezone.utc)
        measurement_time = gps_time - timedelta(seconds=GPS_MINUS_UTC_SECONDS)
        times.append(measurement_time.timestamp())
        latitudes.append(float(position["latitude(degrees)"]))
        longitudes.append(float(position["longitude(degrees)"]))
        altitudes.append(float(position["altitude(meters)"]))
        horizontal_accuracy.append(float(position["h_accuracy(meters)"]))
        vertical_accuracy.append(float(position["v_accuracy(meters)"]))
        satellites.append(int(position["n_sats"]))
        flags.append(int(position["flags"]))
        speeds.append(float(velocity["speed(m/s)"]))
        receive_latencies.append(pc_time.timestamp() - measurement_time.timestamp())

    arrays = {
        "times": np.asarray(times),
        "latitude": np.asarray(latitudes),
        "longitude": np.asarray(longitudes),
        "altitude": np.asarray(altitudes),
        "h_accuracy": np.asarray(horizontal_accuracy),
        "v_accuracy": np.asarray(vertical_accuracy),
        "satellites": np.asarray(satellites),
        "flags": np.asarray(flags),
        "speed": np.asarray(speeds),
        "receive_latency": np.asarray(receive_latencies),
    }
    order = np.argsort(arrays["times"])
    arrays = {key: value[order] for key, value in arrays.items()}

    # Use the first Fixed RTK point as the local ENU origin. / 使用首个固定RTK点作为本地ENU原点。
    fixed_indices = np.flatnonzero(arrays["flags"] == 4)
    if len(fixed_indices) == 0:
        raise ValueError("No Fixed RTK samples / 没有固定 RTK 样本")
    origin_index = int(fixed_indices[0])
    ecef = geodetic_to_ecef(
        arrays["latitude"], arrays["longitude"], arrays["altitude"]
    )
    arrays["enu"] = ecef_to_local_enu(
        ecef,
        float(arrays["latitude"][origin_index]),
        float(arrays["longitude"][origin_index]),
        ecef[origin_index],
    )
    return arrays


def nearest_indices(query_times: np.ndarray, reference_times: np.ndarray) -> np.ndarray:
    """Find the closest GPS row for categorical fields. / 为分类字段寻找最近GPS行。"""
    right = np.searchsorted(reference_times, query_times, side="left")
    right = np.clip(right, 0, len(reference_times) - 1)
    left = np.clip(right - 1, 0, len(reference_times) - 1)
    choose_left = np.abs(query_times - reference_times[left]) <= np.abs(
        reference_times[right] - query_times
    )
    return np.where(choose_left, left, right)


def match_gps_to_kiss(
    kiss_times: np.ndarray,
    gps: dict[str, np.ndarray],
) -> tuple[np.ndarray, dict[str, np.ndarray]]:
    """Interpolate continuous GPS fields at KISS timestamps. / 在KISS时间戳插值连续GPS字段。"""
    valid = (kiss_times >= gps["times"][0]) & (kiss_times <= gps["times"][-1])
    times = kiss_times[valid]
    nearest = nearest_indices(times, gps["times"])
    matched = {
        "times": times,
        "enu": np.column_stack(
            [np.interp(times, gps["times"], gps["enu"][:, axis]) for axis in range(3)]
        ),
        # Accuracy and state belong to the nearest receiver solution.
        # 精度和状态属于最近的一次接收机解算，不跨状态做插值。
        "h_accuracy": gps["h_accuracy"][nearest],
        "v_accuracy": gps["v_accuracy"][nearest],
        "speed": np.interp(times, gps["times"], gps["speed"]),
        "receive_latency": gps["receive_latency"][nearest],
        "flags": gps["flags"][nearest],
        "satellites": gps["satellites"][nearest],
    }
    return valid, matched


def contiguous_fixed_window(flags: np.ndarray) -> np.ndarray:
    """Select the longest continuous Fixed RTK run. / 选择最长连续固定RTK区间。"""
    fixed = flags == 4
    best_start = 0
    best_end = 0
    run_start: int | None = None
    for index, is_fixed in enumerate(np.append(fixed, False)):
        if is_fixed and run_start is None:
            run_start = index
        elif not is_fixed and run_start is not None:
            if index - run_start > best_end - best_start:
                best_start, best_end = run_start, index
            run_start = None
    selected = np.zeros(len(flags), dtype=bool)
    selected[best_start:best_end] = True
    return selected


def metric_block(
    times: np.ndarray,
    estimate: np.ndarray,
    reference: np.ndarray,
) -> tuple[dict[str, float | int], np.ndarray, np.ndarray, np.ndarray]:
    """Align without scale and calculate ATE/RPE. / 不缩放对齐并计算ATE与RPE。"""
    aligned, rotation, translation = rigid_align_2d(estimate, reference)
    errors = np.linalg.norm(aligned - reference, axis=1)
    relative = one_second_relative_errors(times, aligned, reference)
    reference_length = path_length(reference)
    estimate_length = path_length(aligned)
    metrics: dict[str, float | int] = {
        "samples": int(len(times)),
        "duration_s": float(times[-1] - times[0]),
        "reference_path_length_m": reference_length,
        "kiss_path_length_m": estimate_length,
        "path_length_difference_percent": float(
            100.0 * abs(estimate_length - reference_length) / reference_length
        ),
        "ate_mean_m": float(errors.mean()),
        "ate_rmse_m": float(np.sqrt(np.mean(errors**2))),
        "ate_p95_m": float(np.quantile(errors, 0.95)),
        "ate_max_m": float(errors.max()),
        "rpe_1s_samples": int(len(relative)),
        "rpe_1s_mean_m": float(relative.mean()),
        "rpe_1s_rmse_m": float(np.sqrt(np.mean(relative**2))),
        "rpe_1s_p95_m": float(np.quantile(relative, 0.95)),
        "alignment_yaw_degrees": float(
            math.degrees(math.atan2(rotation[1, 0], rotation[0, 0]))
        ),
        "alignment_translation_m": translation.tolist(),
    }
    return metrics, aligned, errors, rotation


def save_detail(
    path: Path,
    kiss_times: np.ndarray,
    kiss_positions: np.ndarray,
    all_aligned: np.ndarray,
    fixed_transform_aligned: np.ndarray,
    matched: dict[str, np.ndarray],
    all_errors: np.ndarray,
    fixed_transform_errors: np.ndarray,
) -> None:
    """Save time-matched RTK details. / 保存按时间匹配的RTK逐点结果。"""
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(
            [
                "stamp_sec",
                "kiss_x_m",
                "kiss_y_m",
                "kiss_z_m",
                "all_gps_aligned_x_m",
                "all_gps_aligned_y_m",
                "fixed_alignment_x_m",
                "fixed_alignment_y_m",
                "gps_east_m",
                "gps_north_m",
                "gps_up_m",
                "gps_flag",
                "gps_flag_name",
                "gps_h_accuracy_m",
                "gps_v_accuracy_m",
                "gps_satellites",
                "gps_speed_mps",
                "gps_receive_latency_s",
                "all_gps_position_difference_m",
                "fixed_alignment_position_difference_m",
                "is_fixed_rtk",
            ]
        )
        for index in range(len(kiss_times)):
            flag = int(matched["flags"][index])
            writer.writerow(
                [
                    kiss_times[index],
                    *kiss_positions[index],
                    *all_aligned[index],
                    *fixed_transform_aligned[index],
                    *matched["enu"][index],
                    flag,
                    GPS_FLAG_NAMES.get(flag, "Unknown / 未知"),
                    matched["h_accuracy"][index],
                    matched["v_accuracy"][index],
                    matched["satellites"][index],
                    matched["speed"][index],
                    matched["receive_latency"][index],
                    all_errors[index],
                    fixed_transform_errors[index],
                    int(flag == 4),
                ]
            )


def parse_args() -> argparse.Namespace:
    """Read optional data and output paths. / 读取可选数据与输出路径。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--position-csv", type=Path, default=DEFAULT_POSITION_CSV)
    parser.add_argument("--velocity-csv", type=Path, default=DEFAULT_VELOCITY_CSV)
    parser.add_argument("--kiss-csv", type=Path, default=DEFAULT_KISS_CSV)
    parser.add_argument("--detail-csv", type=Path, default=DEFAULT_DETAIL_CSV)
    parser.add_argument("--summary-json", type=Path, default=DEFAULT_SUMMARY_JSON)
    parser.add_argument("--full-svg", type=Path, default=DEFAULT_FULL_SVG)
    parser.add_argument("--fixed-svg", type=Path, default=DEFAULT_FIXED_SVG)
    return parser.parse_args()


def main() -> None:
    """Run full-GPS and Fixed-RTK evaluations. / 运行全GPS与固定RTK两种评估。"""
    args = parse_args()
    kiss_times, kiss_positions = load_kiss_trajectory(args.kiss_csv)
    gps = load_gps_reference(args.position_csv, args.velocity_csv)
    valid, matched = match_gps_to_kiss(kiss_times, gps)
    kiss_matched = kiss_positions[valid]
    if len(kiss_matched) < 100:
        raise ValueError(f"Too few matched samples / 匹配样本不足: {len(kiss_matched)}")

    # Evaluate all valid independent GPS states. / 评估全部有效独立GPS状态。
    all_valid = matched["flags"] != 0
    all_metrics, all_aligned_valid, all_errors_valid, _ = metric_block(
        matched["times"][all_valid],
        kiss_matched[all_valid, :2],
        matched["enu"][all_valid, :2],
    )
    all_aligned = np.full((len(kiss_matched), 2), np.nan)
    all_errors = np.full(len(kiss_matched), np.nan)
    all_aligned[all_valid] = all_aligned_valid
    all_errors[all_valid] = all_errors_valid

    # Evaluate the longest continuous centimetre-level Fixed RTK window. / 评估最长连续厘米级固定RTK窗口。
    fixed_window = contiguous_fixed_window(matched["flags"])
    if int(fixed_window.sum()) < 100:
        raise ValueError(
            f"Fixed RTK window too short / 固定RTK区间过短: {int(fixed_window.sum())}"
        )
    fixed_metrics, fixed_aligned, fixed_errors, fixed_rotation = metric_block(
        matched["times"][fixed_window],
        kiss_matched[fixed_window, :2],
        matched["enu"][fixed_window, :2],
    )
    fixed_source_center = kiss_matched[fixed_window, :2].mean(axis=0)
    fixed_target_center = matched["enu"][fixed_window, :2].mean(axis=0)
    fixed_translation = fixed_target_center - fixed_rotation @ fixed_source_center
    fixed_transform_aligned = (
        fixed_rotation @ kiss_matched[:, :2].T
    ).T + fixed_translation
    fixed_transform_errors = np.linalg.norm(
        fixed_transform_aligned - matched["enu"][:, :2], axis=1
    )

    flag_counts = Counter(int(value) for value in matched["flags"])
    total_flags = sum(flag_counts.values())
    # These are project baseline gates, not claims about RTK sensor accuracy.
    # 这些是项目基线验收门槛，不代表RTK传感器本身的精度声明。
    acceptance_criteria = {
        "minimum_fixed_samples": 300,
        "minimum_fixed_duration_s": 30.0,
        "maximum_fixed_h_accuracy_m": 0.05,
        "maximum_path_length_difference_percent": 2.0,
        "maximum_fixed_ate_rmse_m": 5.0,
        "maximum_fixed_rpe_1s_rmse_m": 1.0,
    }
    fixed_h_accuracy_max = float(np.max(matched["h_accuracy"][fixed_window]))
    acceptance_checks = {
        "minimum_fixed_samples": bool(
            int(fixed_metrics["samples"])
            >= int(acceptance_criteria["minimum_fixed_samples"])
        ),
        "minimum_fixed_duration_s": bool(
            float(fixed_metrics["duration_s"])
            >= float(acceptance_criteria["minimum_fixed_duration_s"])
        ),
        "maximum_fixed_h_accuracy_m": bool(
            fixed_h_accuracy_max
            <= float(acceptance_criteria["maximum_fixed_h_accuracy_m"])
        ),
        "maximum_path_length_difference_percent": bool(
            float(fixed_metrics["path_length_difference_percent"])
            <= float(acceptance_criteria["maximum_path_length_difference_percent"])
        ),
        "maximum_fixed_ate_rmse_m": bool(
            float(fixed_metrics["ate_rmse_m"])
            <= float(acceptance_criteria["maximum_fixed_ate_rmse_m"])
        ),
        "maximum_fixed_rpe_1s_rmse_m": bool(
            float(fixed_metrics["rpe_1s_rmse_m"])
            <= float(acceptance_criteria["maximum_fixed_rpe_1s_rmse_m"])
        ),
    }
    acceptance_passed = all(acceptance_checks.values())
    summary = {
        "dataset": "DTU Roadrunners Autonomous Ecocar Race Track Dataset",
        "sequence": "03-2020 RRC/2020-03-11-15-39.bag",
        "sensor": "Velodyne VLP-16 and Swift Piksi Multi RTK GPS",
        "comparison": "KISS-ICP versus independent GPS/RTK position",
        "time_sync_method": "GPS measurement time converted to UTC with the 18-second GPS-UTC offset; PC receive latency is excluded",
        "median_gps_receive_latency_s": float(
            np.median(matched["receive_latency"])
        ),
        "p95_gps_receive_latency_s": float(
            np.quantile(matched["receive_latency"], 0.95)
        ),
        "coordinate_method": "WGS-84 geodetic to ECEF to local ENU",
        "alignment_method": "SE(2) rotation and translation only; no scale",
        "gps_state_counts": {
            GPS_FLAG_NAMES.get(flag, str(flag)): count
            for flag, count in sorted(flag_counts.items())
        },
        "gps_state_percent": {
            GPS_FLAG_NAMES.get(flag, str(flag)): 100.0 * count / total_flags
            for flag, count in sorted(flag_counts.items())
        },
        "all_valid_gps": all_metrics,
        "fixed_rtk_window": {
            **fixed_metrics,
            "start_offset_s": float(
                matched["times"][fixed_window][0] - matched["times"][0]
            ),
            "end_offset_s": float(
                matched["times"][fixed_window][-1] - matched["times"][0]
            ),
            "median_horizontal_accuracy_m": float(
                np.median(matched["h_accuracy"][fixed_window])
            ),
            "max_horizontal_accuracy_m": float(
                fixed_h_accuracy_max
            ),
        },
        "acceptance_criteria": acceptance_criteria,
        "acceptance_checks": acceptance_checks,
        "acceptance_passed": acceptance_passed,
        "note": "Fixed RTK metrics are the high-accuracy result; all-GPS metrics mix SPP, Float RTK, Fixed RTK, and SBAS",
    }
    args.summary_json.parent.mkdir(parents=True, exist_ok=True)
    args.summary_json.write_text(
        json.dumps(summary, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    save_detail(
        args.detail_csv,
        matched["times"],
        kiss_matched,
        all_aligned,
        fixed_transform_aligned,
        matched,
        all_errors,
        fixed_transform_errors,
    )
    save_trajectory_svg(
        args.full_svg,
        all_aligned_valid,
        matched["enu"][all_valid, :2],
        float(all_metrics["ate_rmse_m"]),
    )
    save_trajectory_svg(
        args.fixed_svg,
        fixed_aligned,
        matched["enu"][fixed_window, :2],
        float(fixed_metrics["ate_rmse_m"]),
    )

    from export_bilingual_reports import export_all

    export_all(PROJECT_ROOT / "output", PROJECT_ROOT / "output" / "bilingual")

    print(f"Matched samples / 匹配样本：{len(kiss_matched)}")
    print(
        "Fixed RTK coverage / 固定RTK覆盖："
        f"{flag_counts.get(4, 0)} / {total_flags} "
        f"({100.0 * flag_counts.get(4, 0) / total_flags:.1f}%)"
    )
    print(
        "Fixed RTK ATE RMSE / 固定RTK绝对轨迹误差RMSE："
        f"{fixed_metrics['ate_rmse_m']:.3f} m"
    )
    print(
        "Fixed RTK RPE 1 s RMSE / 固定RTK一秒相对误差RMSE："
        f"{fixed_metrics['rpe_1s_rmse_m']:.3f} m"
    )
    print(
        "All-GPS ATE RMSE / 全GPS绝对轨迹误差RMSE："
        f"{all_metrics['ate_rmse_m']:.3f} m"
    )
    print(
        "Automated baseline / 自动基线验收："
        f"{'PASS / 通过' if acceptance_passed else 'FAIL / 未通过'}"
    )
    print(f"Summary / 汇总：{args.summary_json}")
    if not acceptance_passed:
        raise RuntimeError(
            "DTU RTK baseline checks failed / DTU RTK 基线验收未通过"
        )
    print("DTU_RTK_TRAJECTORY_EVALUATION_OK / DTU RTK 轨迹评估成功")


if __name__ == "__main__":
    main()
