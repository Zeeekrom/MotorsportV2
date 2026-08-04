"""Evaluate H4 closed-loop lap and relocalization logs. / 评估H4闭环整圈和重定位日志。"""

from __future__ import annotations

import argparse
import csv
import json
import math
import re
import sys
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_LOG = PROJECT_ROOT / "output" / "h4_closed_loop_lap_test.log"
DEFAULT_CSV = PROJECT_ROOT / "output" / "h4_closed_loop_lap_evaluation.csv"
DEFAULT_JSON = PROJECT_ROOT / "output" / "h4_closed_loop_lap_evaluation.json"

ROW_PATTERN = re.compile(
    r"frame / 帧=(?P<frame>\d+).*?"
    r"map_cones / 地图锥桶=(?P<map>\d+).*?"
    r"path_points / 路径点=(?P<path>\d+).*?"
    r"control / 控制=(?P<control>active|waiting).*?"
    r"vehicle_progress / 车辆进度=(?P<progress>-?[0-9.]+)m.*?"
    r"cross_track_error / 横向误差=(?P<cross>-?[0-9.]+)m.*?"
    r"heading_error / 航向误差=(?P<heading>-?[0-9.]+)deg.*?"
    r"vehicle_speed / 车辆速度=(?P<speed>[0-9.]+)mps.*?"
    r"steer / 转向=(?P<steer>-?[0-9.]+).*?"
    r"lap_progress / 圈进度=(?P<lap_progress>-?[0-9.]+)%.*?"
    r"lap_count / 完成圈数=(?P<lap_count>\d+).*?"
    r"start_distance / 起点距离=(?P<start_distance>[0-9.]+)m.*?"
    r"raw_localization_error / 原始定位误差=(?P<raw_error>[0-9.]+)m.*?"
    r"corrected_localization_error / 校正定位误差="
    r"(?P<corrected_error>[0-9.]+)m.*?"
    r"relocalization_matches / 重定位匹配=(?P<matches>\d+).*?"
    r"relocalization_updates / 重定位更新=(?P<updates>\d+)"
)
TARGET_PATTERN = re.compile(
    r"Target lap reached / 已达到目标圈数: "
    r"laps / 圈数=(?P<laps>\d+), "
    r"progress / 进度=(?P<progress>[0-9.]+)%"
)

if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
if hasattr(sys.stderr, "reconfigure"):
    sys.stderr.reconfigure(encoding="utf-8", errors="replace")


def percentile(values: list[float], fraction: float) -> float:
    """Calculate a dependency-free linear percentile. / 计算无额外依赖的线性百分位数。"""
    ordered = sorted(values)
    if not ordered:
        return math.nan
    position = (len(ordered) - 1) * fraction
    lower = int(math.floor(position))
    upper = int(math.ceil(position))
    if lower == upper:
        return ordered[lower]
    weight = position - lower
    return ordered[lower] * (1.0 - weight) + ordered[upper] * weight


def parse_log(path: Path) -> list[dict[str, int | float | str]]:
    """Extract periodic H4 observations. / 提取H4周期性观测。"""
    rows: list[dict[str, int | float | str]] = []
    for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
        target_match = TARGET_PATTERN.search(line)
        if target_match is not None and rows:
            # The terminal event may occur between 20-frame observation logs.
            # 终点事件可能发生在每20帧一次的观测日志之间。
            terminal_row = rows[-1].copy()
            terminal_row["completed_laps"] = int(
                target_match.group("laps")
            )
            terminal_row["lap_progress_percent"] = float(
                target_match.group("progress")
            )
            rows.append(terminal_row)
            continue
        match = ROW_PATTERN.search(line)
        if match is None:
            continue
        rows.append(
            {
                "frame": int(match.group("frame")),
                "map_cones": int(match.group("map")),
                "path_points": int(match.group("path")),
                "control_state": match.group("control"),
                "vehicle_progress_m": float(match.group("progress")),
                "cross_track_error_m": abs(float(match.group("cross"))),
                "heading_error_degrees": abs(float(match.group("heading"))),
                "vehicle_speed_mps": float(match.group("speed")),
                "carla_steer": float(match.group("steer")),
                "lap_progress_percent": float(match.group("lap_progress")),
                "completed_laps": int(match.group("lap_count")),
                "distance_to_start_m": float(match.group("start_distance")),
                "raw_localization_error_m": float(match.group("raw_error")),
                "corrected_localization_error_m": float(
                    match.group("corrected_error")
                ),
                "relocalization_matches": int(match.group("matches")),
                "relocalization_updates": int(match.group("updates")),
            }
        )
    return rows


def mean(values: list[float]) -> float:
    """Return a safe arithmetic mean. / 返回安全的算术平均值。"""
    return sum(values) / len(values) if values else math.nan


def evaluate(rows: list[dict[str, int | float | str]]) -> dict[str, object]:
    """Build H4 metrics and automatic acceptance checks. / 建立H4指标和自动验收检查。"""
    if not rows:
        raise ValueError(
            "No H4 observation rows found. / 没有找到H4观测记录。"
        )
    active = [row for row in rows if row["control_state"] == "active"]
    localized = [
        row for row in active if int(row["relocalization_updates"]) > 0
    ]
    cross_errors = [float(row["cross_track_error_m"]) for row in active]
    heading_errors = [
        float(row["heading_error_degrees"]) for row in active
    ]
    raw_errors = [
        float(row["raw_localization_error_m"]) for row in localized
    ]
    corrected_errors = [
        float(row["corrected_localization_error_m"]) for row in localized
    ]
    mean_raw = mean(raw_errors)
    mean_corrected = mean(corrected_errors)
    improvement = (
        100.0 * (1.0 - mean_corrected / mean_raw)
        if mean_raw > 1e-9
        else 0.0
    )
    metrics = {
        "observations": len(rows),
        "active_control_observations": len(active),
        "localized_observations": len(localized),
        "maximum_map_cones": max(int(row["map_cones"]) for row in rows),
        "maximum_path_points": max(int(row["path_points"]) for row in rows),
        "maximum_progress_m": max(
            float(row["vehicle_progress_m"]) for row in rows
        ),
        "maximum_lap_progress_percent": max(
            float(row["lap_progress_percent"]) for row in rows
        ),
        "completed_laps": max(int(row["completed_laps"]) for row in rows),
        "final_distance_to_start_m": float(rows[-1]["distance_to_start_m"]),
        "mean_cross_track_error_m": mean(cross_errors),
        "p95_cross_track_error_m": percentile(cross_errors, 0.95),
        "maximum_cross_track_error_m": max(cross_errors, default=math.nan),
        "mean_heading_error_degrees": mean(heading_errors),
        "p95_heading_error_degrees": percentile(heading_errors, 0.95),
        "mean_active_speed_mps": mean(
            [float(row["vehicle_speed_mps"]) for row in active]
        ),
        "maximum_raw_localization_error_m": max(
            raw_errors,
            default=math.nan,
        ),
        "mean_raw_localization_error_m": mean_raw,
        "mean_corrected_localization_error_m": mean_corrected,
        "p95_corrected_localization_error_m": percentile(
            corrected_errors,
            0.95,
        ),
        "localization_improvement_percent": improvement,
        "maximum_relocalization_matches": max(
            int(row["relocalization_matches"]) for row in rows
        ),
        "relocalization_updates": max(
            int(row["relocalization_updates"]) for row in rows
        ),
    }
    criteria = {
        "minimum_observations": 40,
        "minimum_active_control_observations": 25,
        "minimum_map_cones": 20,
        "minimum_lap_progress_percent": 99.0,
        "minimum_completed_laps": 1,
        "maximum_final_distance_to_start_m": 2.5,
        "maximum_mean_cross_track_error_m": 1.0,
        "maximum_p95_cross_track_error_m": 1.5,
        "maximum_p95_heading_error_degrees": 15.0,
        "minimum_raw_localization_error_m": 0.50,
        "maximum_mean_corrected_localization_error_m": 0.35,
        "maximum_p95_corrected_localization_error_m": 0.60,
        "minimum_localization_improvement_percent": 40.0,
        "minimum_relocalization_updates": 10,
    }
    checks = {
        "minimum_observations": metrics["observations"]
        >= criteria["minimum_observations"],
        "minimum_active_control_observations": metrics[
            "active_control_observations"
        ]
        >= criteria["minimum_active_control_observations"],
        "minimum_map_cones": metrics["maximum_map_cones"]
        >= criteria["minimum_map_cones"],
        "minimum_lap_progress_percent": metrics[
            "maximum_lap_progress_percent"
        ]
        >= criteria["minimum_lap_progress_percent"],
        "minimum_completed_laps": metrics["completed_laps"]
        >= criteria["minimum_completed_laps"],
        "maximum_final_distance_to_start_m": metrics[
            "final_distance_to_start_m"
        ]
        <= criteria["maximum_final_distance_to_start_m"],
        "maximum_mean_cross_track_error_m": metrics[
            "mean_cross_track_error_m"
        ]
        <= criteria["maximum_mean_cross_track_error_m"],
        "maximum_p95_cross_track_error_m": metrics[
            "p95_cross_track_error_m"
        ]
        <= criteria["maximum_p95_cross_track_error_m"],
        "maximum_p95_heading_error_degrees": metrics[
            "p95_heading_error_degrees"
        ]
        <= criteria["maximum_p95_heading_error_degrees"],
        "minimum_raw_localization_error_m": metrics[
            "maximum_raw_localization_error_m"
        ]
        >= criteria["minimum_raw_localization_error_m"],
        "maximum_mean_corrected_localization_error_m": metrics[
            "mean_corrected_localization_error_m"
        ]
        <= criteria["maximum_mean_corrected_localization_error_m"],
        "maximum_p95_corrected_localization_error_m": metrics[
            "p95_corrected_localization_error_m"
        ]
        <= criteria["maximum_p95_corrected_localization_error_m"],
        "minimum_localization_improvement_percent": metrics[
            "localization_improvement_percent"
        ]
        >= criteria["minimum_localization_improvement_percent"],
        "minimum_relocalization_updates": metrics["relocalization_updates"]
        >= criteria["minimum_relocalization_updates"],
    }
    return {
        "stage": "H4 full closed-track lap and continuous relocalization",
        "scenario": "Circular cone track with deterministic bicycle model",
        "localization": (
            "Synthetic odometry drift corrected against a prebuilt "
            "persistent cone-landmark map"
        ),
        "metrics": metrics,
        "acceptance_criteria": criteria,
        "acceptance_checks": checks,
        "acceptance_passed": all(checks.values()),
        "note": (
            "H3 already validated CARLA vehicle dynamics. H4 represents a "
            "second run on a pre-mapped course and isolates full-lap live "
            "perception, map refinement, relocalization, planning, and control."
        ),
    }


def write_csv(
    rows: list[dict[str, int | float | str]],
    path: Path,
) -> None:
    """Write all H4 observations. / 写出全部H4观测。"""
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8-sig", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def main() -> None:
    """Parse arguments, evaluate, and save artifacts. / 解析参数、评估并保存产物。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--log", type=Path, default=DEFAULT_LOG)
    parser.add_argument("--csv", type=Path, default=DEFAULT_CSV)
    parser.add_argument("--json", type=Path, default=DEFAULT_JSON)
    arguments = parser.parse_args()
    rows = parse_log(arguments.log)
    summary = evaluate(rows)
    write_csv(rows, arguments.csv)
    arguments.json.parent.mkdir(parents=True, exist_ok=True)
    arguments.json.write_text(
        json.dumps(summary, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    print(
        f"Observations / 观测数：{len(rows)}\n"
        f"Completed laps / 完成圈数："
        f"{summary['metrics']['completed_laps']}\n"
        f"Mean cross-track error / 平均横向误差："
        f"{summary['metrics']['mean_cross_track_error_m']:.3f} m\n"
        f"Mean corrected localization error / 平均校正定位误差："
        f"{summary['metrics']['mean_corrected_localization_error_m']:.3f} m\n"
        f"Acceptance passed / 验收通过：{summary['acceptance_passed']}"
    )
    if not summary["acceptance_passed"]:
        failed = [
            name
            for name, passed in summary["acceptance_checks"].items()
            if not passed
        ]
        print("Failed checks / 未通过检查：" + ", ".join(failed))
        raise SystemExit(2)
    print("H4_CLOSED_LOOP_EVALUATION_OK / H4闭环整圈评估成功")


if __name__ == "__main__":
    main()
