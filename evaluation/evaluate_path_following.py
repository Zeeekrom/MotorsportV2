"""Evaluate H3 closed-loop path following from the ROS/CARLA log. / 根据ROS/CARLA日志评估H3闭环路径跟随。"""

from __future__ import annotations

import argparse
import csv
import json
import math
import re
import sys
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_LOG = PROJECT_ROOT / "output" / "h3_path_following_test.log"
DEFAULT_CSV = PROJECT_ROOT / "output" / "h3_path_following_evaluation.csv"
DEFAULT_JSON = PROJECT_ROOT / "output" / "h3_path_following_evaluation.json"

ROW_PATTERN = re.compile(
    r"frame / 帧=(?P<frame>\d+).*?"
    r"map_cones / 地图锥桶=(?P<map>\d+).*?"
    r"path_points / 路径点=(?P<path>\d+).*?"
    r"control / 控制=(?P<control>active|waiting).*?"
    r"vehicle_progress / 车辆进度=(?P<progress>-?[0-9.]+)m.*?"
    r"cross_track_error / 横向误差=(?P<cross>-?[0-9.]+)m.*?"
    r"heading_error / 航向误差=(?P<heading>-?[0-9.]+)deg.*?"
    r"vehicle_speed / 车辆速度=(?P<speed>[0-9.]+)mps.*?"
    r"steer / 转向=(?P<steer>-?[0-9.]+)"
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
    """Extract periodic closed-loop observations. / 提取周期性闭环观测。"""
    rows: list[dict[str, int | float | str]] = []
    for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
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
            }
        )
    if not rows:
        raise ValueError(
            f"No H3 observations found / 没有找到H3观测: {path}"
        )
    return rows


def save_csv(path: Path, rows: list[dict[str, int | float | str]]) -> None:
    """Write the closed-loop observations. / 写入闭环逐次观测。"""
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def evaluate(rows: list[dict[str, int | float | str]]) -> dict[str, object]:
    """Calculate control metrics and acceptance checks. / 计算控制指标与验收检查。"""
    active = [row for row in rows if row["control_state"] == "active"]
    if not active:
        raise ValueError("Controller never became active / 控制器从未进入活动状态")
    cross_errors = [float(row["cross_track_error_m"]) for row in active]
    heading_errors = [float(row["heading_error_degrees"]) for row in active]
    speeds = [float(row["vehicle_speed_mps"]) for row in active]
    progress_values = [float(row["vehicle_progress_m"]) for row in rows]
    active_progress = [float(row["vehicle_progress_m"]) for row in active]

    metrics = {
        "observations": len(rows),
        "active_control_observations": len(active),
        "active_control_percent": 100.0 * len(active) / len(rows),
        "maximum_map_cones": max(int(row["map_cones"]) for row in rows),
        "maximum_path_points": max(int(row["path_points"]) for row in rows),
        "maximum_progress_m": max(progress_values),
        "active_progress_gain_m": max(active_progress) - min(active_progress),
        "mean_cross_track_error_m": sum(cross_errors) / len(cross_errors),
        "p95_cross_track_error_m": percentile(cross_errors, 0.95),
        "maximum_cross_track_error_m": max(cross_errors),
        "mean_heading_error_degrees": sum(heading_errors) / len(heading_errors),
        "p95_heading_error_degrees": percentile(heading_errors, 0.95),
        "mean_active_speed_mps": sum(speeds) / len(speeds),
        "maximum_active_speed_mps": max(speeds),
        "maximum_absolute_steer": max(
            abs(float(row["carla_steer"])) for row in active
        ),
    }
    criteria = {
        "minimum_observations": 20,
        "minimum_active_control_observations": 10,
        "minimum_active_progress_gain_m": 15.0,
        "maximum_mean_cross_track_error_m": 1.0,
        "maximum_p95_cross_track_error_m": 1.5,
        "maximum_mean_heading_error_degrees": 12.0,
        "maximum_active_speed_mps": 3.0,
    }
    checks = {
        "minimum_observations": metrics["observations"]
        >= criteria["minimum_observations"],
        "minimum_active_control_observations": metrics[
            "active_control_observations"
        ]
        >= criteria["minimum_active_control_observations"],
        "minimum_active_progress_gain_m": metrics["active_progress_gain_m"]
        >= criteria["minimum_active_progress_gain_m"],
        "maximum_mean_cross_track_error_m": metrics[
            "mean_cross_track_error_m"
        ]
        <= criteria["maximum_mean_cross_track_error_m"],
        "maximum_p95_cross_track_error_m": metrics[
            "p95_cross_track_error_m"
        ]
        <= criteria["maximum_p95_cross_track_error_m"],
        "maximum_mean_heading_error_degrees": metrics[
            "mean_heading_error_degrees"
        ]
        <= criteria["maximum_mean_heading_error_degrees"],
        "maximum_active_speed_mps": metrics["maximum_active_speed_mps"]
        <= criteria["maximum_active_speed_mps"],
    }
    return {
        "stage": "H3 closed-loop centre-path following",
        "controller": "Pure pursuit with curvature-based speed reduction",
        "command_topic": "/path_following/cmd_vel",
        "command_type": "geometry_msgs/Twist",
        "metrics": metrics,
        "acceptance_criteria": criteria,
        "acceptance_checks": checks,
        "acceptance_passed": all(checks.values()),
        "note": "CARLA steering sign is converted from ROS left-positive yaw to CARLA right-positive steer",
    }


def parse_args() -> argparse.Namespace:
    """Read optional input and output paths. / 读取可选输入与输出路径。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--log", type=Path, default=DEFAULT_LOG)
    parser.add_argument("--csv", type=Path, default=DEFAULT_CSV)
    parser.add_argument("--json", type=Path, default=DEFAULT_JSON)
    return parser.parse_args()


def main() -> None:
    """Parse, score, save, and enforce H3 acceptance. / 解析、评分、保存并执行H3验收。"""
    args = parse_args()
    rows = parse_log(args.log)
    save_csv(args.csv, rows)
    summary = evaluate(rows)
    args.json.parent.mkdir(parents=True, exist_ok=True)
    args.json.write_text(
        json.dumps(summary, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )

    metrics = summary["metrics"]
    print(
        "Active progress / 活动控制进度："
        f"{metrics['active_progress_gain_m']:.3f} m"
    )
    print(
        "Mean cross-track error / 平均横向误差："
        f"{metrics['mean_cross_track_error_m']:.3f} m"
    )
    print(
        "P95 cross-track error / 横向误差P95："
        f"{metrics['p95_cross_track_error_m']:.3f} m"
    )
    if not summary["acceptance_passed"]:
        raise RuntimeError("H3 baseline failed / H3基线验收失败")
    print("H3_PATH_FOLLOWING_OK / H3路径跟随测试成功")


if __name__ == "__main__":
    main()
