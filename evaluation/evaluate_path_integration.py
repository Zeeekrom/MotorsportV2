"""Summarise the H1 CARLA-to-Path integration log. / 汇总H1 CARLA到Path集成日志。"""

from __future__ import annotations

import argparse
import csv
import json
import re
import sys
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_LOG = PROJECT_ROOT / "output" / "h1_path_integration_test.log"
DEFAULT_CSV = PROJECT_ROOT / "output" / "h1_path_integration_evaluation.csv"
DEFAULT_JSON = PROJECT_ROOT / "output" / "h1_path_integration_evaluation.json"

FRAME_PATTERN = re.compile(
    r"frame / 帧=(?P<frame>\d+).*?"
    r"stable / 稳定锥桶=(?P<stable>\d+).*?"
    r"map_cones / 地图锥桶=(?P<map>\d+).*?"
    r"path_points / 路径点=(?P<path>\d+).*?"
    r"path_truth_error / 路径真值误差="
    r"(?P<path_error>nanm|[0-9.]+m).*?"
    r"marker_truth_matches / 标记真值匹配="
    r"(?P<matches>\d+)/(?P<tracks>\d+).*?"
    r"matched_error / 匹配误差=(?P<error>n/a|[0-9.]+m)"
)

if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
if hasattr(sys.stderr, "reconfigure"):
    sys.stderr.reconfigure(encoding="utf-8", errors="replace")


def parse_log(path: Path) -> list[dict[str, int | float | None]]:
    """Extract periodic detector/path observations. / 提取周期性的检测与路径观测。"""
    observations: list[dict[str, int | float | None]] = []
    for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
        match = FRAME_PATTERN.search(line)
        if match is None:
            continue
        error_text = match.group("error")
        observations.append(
            {
                "frame": int(match.group("frame")),
                "stable_cones": int(match.group("stable")),
                "map_cones": int(match.group("map")),
                "path_points": int(match.group("path")),
                "path_truth_error_m": (
                    None
                    if match.group("path_error") == "nanm"
                    else float(match.group("path_error")[:-1])
                ),
                "marker_truth_matches": int(match.group("matches")),
                "marker_tracks": int(match.group("tracks")),
                "matched_error_m": (
                    None if error_text == "n/a" else float(error_text[:-1])
                ),
            }
        )
    if not observations:
        raise ValueError(
            f"No H1 observations found / 没有找到H1观测: {path}"
        )
    return observations


def save_csv(
    path: Path,
    observations: list[dict[str, int | float | None]],
) -> None:
    """Write machine-stable per-observation rows. / 写入字段稳定的逐观测结果。"""
    path.parent.mkdir(parents=True, exist_ok=True)
    fields = [
        "frame",
        "stable_cones",
        "map_cones",
        "path_points",
        "path_truth_error_m",
        "marker_truth_matches",
        "marker_tracks",
        "matched_error_m",
    ]
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        writer.writerows(observations)


def build_summary(
    observations: list[dict[str, int | float | None]],
    *,
    stage: str,
    limitation: str,
    minimum_map_cones: int,
    minimum_maximum_path_points: int,
    minimum_valid_path_observations: int,
    maximum_mean_path_truth_error_m: float | None,
) -> dict[str, object]:
    """Calculate report metrics and acceptance checks. / 计算报告指标与验收检查。"""
    errors = [
        float(row["matched_error_m"])
        for row in observations
        if row["matched_error_m"] is not None
    ]
    valid_paths = [
        int(row["path_points"])
        for row in observations
        if int(row["path_points"]) >= 2
    ]
    path_truth_errors = [
        float(row["path_truth_error_m"])
        for row in observations
        if row["path_truth_error_m"] is not None
    ]
    total_tracks = sum(int(row["marker_tracks"]) for row in observations)
    total_matches = sum(
        int(row["marker_truth_matches"]) for row in observations
    )
    metrics = {
        "observations": len(observations),
        "maximum_stable_cones": max(
            int(row["stable_cones"]) for row in observations
        ),
        "maximum_map_cones": max(
            int(row["map_cones"]) for row in observations
        ),
        "maximum_path_points": max(
            int(row["path_points"]) for row in observations
        ),
        "valid_path_observations": len(valid_paths),
        "mean_valid_path_points": (
            sum(valid_paths) / len(valid_paths) if valid_paths else 0.0
        ),
        "marker_truth_match_percent": (
            100.0 * total_matches / total_tracks if total_tracks else 0.0
        ),
        "mean_matched_cone_error_m": (
            sum(errors) / len(errors) if errors else None
        ),
        "mean_path_truth_error_m": (
            sum(path_truth_errors) / len(path_truth_errors)
            if path_truth_errors
            else None
        ),
    }
    criteria = {
        "minimum_observations": 10,
        "minimum_maximum_stable_cones": 8,
        "minimum_maximum_path_points": int(minimum_maximum_path_points),
        "minimum_valid_path_observations": int(
            minimum_valid_path_observations
        ),
        "maximum_mean_matched_cone_error_m": 0.50,
        "minimum_maximum_map_cones": int(minimum_map_cones),
    }
    checks = {
        "minimum_observations": metrics["observations"]
        >= criteria["minimum_observations"],
        "minimum_maximum_stable_cones": metrics["maximum_stable_cones"]
        >= criteria["minimum_maximum_stable_cones"],
        "minimum_maximum_path_points": metrics["maximum_path_points"]
        >= criteria["minimum_maximum_path_points"],
        "minimum_valid_path_observations": metrics["valid_path_observations"]
        >= criteria["minimum_valid_path_observations"],
        "maximum_mean_matched_cone_error_m": (
            metrics["mean_matched_cone_error_m"] is not None
            and metrics["mean_matched_cone_error_m"]
            <= criteria["maximum_mean_matched_cone_error_m"]
        ),
        "minimum_maximum_map_cones": metrics["maximum_map_cones"]
        >= criteria["minimum_maximum_map_cones"],
    }
    if maximum_mean_path_truth_error_m is not None:
        criteria["maximum_mean_path_truth_error_m"] = float(
            maximum_mean_path_truth_error_m
        )
        checks["maximum_mean_path_truth_error_m"] = (
            metrics["mean_path_truth_error_m"] is not None
            and metrics["mean_path_truth_error_m"]
            <= criteria["maximum_mean_path_truth_error_m"]
        )
    return {
        "stage": stage,
        "topics": {
            "input": "/detected_cones",
            "map": "/cone_map",
            "output": "/planned_path",
            "output_type": "nav_msgs/Path",
        },
        "planner": "Sequential cross-track pairing with heading updates and a 2-8 m width gate",
        "metrics": metrics,
        "acceptance_criteria": criteria,
        "acceptance_checks": checks,
        "acceptance_passed": all(checks.values()),
        "limitation": limitation,
    }


def parse_args() -> argparse.Namespace:
    """Read optional file paths. / 读取可选文件路径。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--log", type=Path, default=DEFAULT_LOG)
    parser.add_argument("--csv", type=Path, default=DEFAULT_CSV)
    parser.add_argument("--json", type=Path, default=DEFAULT_JSON)
    parser.add_argument(
        "--stage",
        default="H1 cone-map centre-path interface",
    )
    parser.add_argument(
        "--limitation",
        default="Current baseline assumes a vehicle-aligned straight or mild-curve local corridor",
    )
    parser.add_argument("--minimum-map-cones", type=int, default=0)
    parser.add_argument(
        "--minimum-maximum-path-points",
        type=int,
        default=5,
    )
    parser.add_argument(
        "--minimum-valid-path-observations",
        type=int,
        default=5,
    )
    parser.add_argument("--maximum-mean-path-truth-error-m", type=float)
    return parser.parse_args()


def main() -> None:
    """Parse, score, save, and enforce the H1 baseline. / 解析、评分、保存并执行H1基线验收。"""
    args = parse_args()
    observations = parse_log(args.log)
    save_csv(args.csv, observations)
    summary = build_summary(
        observations,
        stage=args.stage,
        limitation=args.limitation,
        minimum_map_cones=args.minimum_map_cones,
        minimum_maximum_path_points=args.minimum_maximum_path_points,
        minimum_valid_path_observations=args.minimum_valid_path_observations,
        maximum_mean_path_truth_error_m=args.maximum_mean_path_truth_error_m,
    )
    args.json.parent.mkdir(parents=True, exist_ok=True)
    args.json.write_text(
        json.dumps(summary, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )

    metrics = summary["metrics"]
    print(
        "Maximum path points / 最大路径点数："
        f"{metrics['maximum_path_points']}"
    )
    print(
        "Valid path observations / 有效路径观测："
        f"{metrics['valid_path_observations']}"
    )
    print(
        "Mean matched cone error / 平均锥桶匹配误差："
        f"{metrics['mean_matched_cone_error_m']:.3f} m"
    )
    if metrics["mean_path_truth_error_m"] is not None:
        print(
            "Mean path truth error / 平均路径真值误差："
            f"{metrics['mean_path_truth_error_m']:.3f} m"
        )
    if not summary["acceptance_passed"]:
        raise RuntimeError(
            "Path integration baseline failed / 路径集成基线验收失败"
        )
    print(
        "PATH_INTEGRATION_EVALUATION_OK / 路径集成评估成功"
    )


if __name__ == "__main__":
    main()
