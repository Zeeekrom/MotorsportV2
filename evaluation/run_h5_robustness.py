"""Run H5 robustness scenarios and save bilingual-ready data. / 运行H5鲁棒性场景并保存双语报告数据。"""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path
import sys

from experiment_registry import record_experiment


PROJECT_ROOT = Path(__file__).resolve().parents[1]
PACKAGE_SOURCE = (
    PROJECT_ROOT
    / "ros2_ws"
    / "src"
    / "motorsport_lidar_bridge"
)
sys.path.insert(0, str(PACKAGE_SOURCE))

from motorsport_lidar_bridge.h5_benchmark import (  # noqa: E402
    default_scenarios,
    evaluate_matrix,
)


DEFAULT_OBSERVATIONS = (
    PROJECT_ROOT / "output" / "h5_robustness_observations.csv"
)
DEFAULT_SCENARIOS = (
    PROJECT_ROOT / "output" / "h5_robustness_scenarios.csv"
)
DEFAULT_SUMMARY = (
    PROJECT_ROOT / "output" / "h5_robustness_evaluation.json"
)

if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
if hasattr(sys.stderr, "reconfigure"):
    sys.stderr.reconfigure(encoding="utf-8", errors="replace")


def write_rows(rows: list[dict[str, object]], path: Path) -> None:
    """Write dictionaries as a UTF-8 CSV. / 把字典写为UTF-8 CSV。"""
    if not rows:
        raise ValueError("No rows to write. / 没有可写入的数据行。")
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8-sig", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def scenario_rows(summary: dict[str, object]) -> list[dict[str, object]]:
    """Flatten scenario summaries for report tables. / 展平场景汇总用于报告表格。"""
    rows = []
    for scenario_summary in summary["scenario_summaries"]:
        scenario = scenario_summary["scenario"]
        metrics = scenario_summary["metrics"]
        rows.append(
            {
                "scenario": scenario["name"],
                "track_shape": scenario["track_shape"],
                "target_laps": scenario["target_laps"],
                "dropout_rate": scenario["dropout_rate"],
                "noise_standard_deviation_m": (
                    scenario["noise_standard_deviation_m"]
                ),
                "localization_drift_per_m": (
                    scenario["localization_drift_per_m"]
                ),
                "dynamic_obstacle": scenario["dynamic_obstacle"],
                "completed_laps": metrics["completed_laps"],
                "completion_time_s": metrics["completion_time_s"],
                "mean_cross_track_error_m": (
                    metrics["mean_cross_track_error_m"]
                ),
                "p95_cross_track_error_m": (
                    metrics["p95_cross_track_error_m"]
                ),
                "mean_corrected_localization_error_m": (
                    metrics["mean_corrected_localization_error_m"]
                ),
                "p95_corrected_localization_error_m": (
                    metrics["p95_corrected_localization_error_m"]
                ),
                "localization_improvement_percent": (
                    metrics["localization_improvement_percent"]
                ),
                "sensor_retention_ratio": (
                    metrics["sensor_retention_ratio"]
                ),
                "stopped_duration_s": metrics["stopped_duration_s"],
                "resumed_after_obstacle": (
                    metrics["resumed_after_obstacle"]
                ),
                "acceptance_passed": (
                    scenario_summary["acceptance_passed"]
                ),
            }
        )
    return rows


def main() -> None:
    """Parse options, run the matrix, and save artifacts. / 解析选项、运行矩阵并保存产物。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--laps",
        type=int,
        default=3,
        help="Target laps per scenario / 每个场景的目标圈数",
    )
    parser.add_argument(
        "--observations",
        type=Path,
        default=DEFAULT_OBSERVATIONS,
    )
    parser.add_argument(
        "--scenarios",
        type=Path,
        default=DEFAULT_SCENARIOS,
    )
    parser.add_argument(
        "--summary",
        type=Path,
        default=DEFAULT_SUMMARY,
    )
    parser.add_argument(
        "--attempt-label",
        default="automatic",
        help="Short run label / 简短运行标签",
    )
    parser.add_argument(
        "--parent-run-id",
        default="",
        help="Previous run identifier / 上一次运行编号",
    )
    parser.add_argument(
        "--change-reason-en",
        default="Automated deterministic H5 regression",
        help="Why parameters or code changed / 参数或代码变化原因",
    )
    parser.add_argument(
        "--change-reason-zh",
        default="自动运行确定性H5回归",
        help="参数或代码变化原因的中文说明 / Chinese change reason",
    )
    arguments = parser.parse_args()
    scenarios = default_scenarios(target_laps=arguments.laps)
    observations, summary = evaluate_matrix(scenarios)
    flattened = scenario_rows(summary)
    write_rows(observations, arguments.observations)
    write_rows(flattened, arguments.scenarios)
    arguments.summary.parent.mkdir(parents=True, exist_ok=True)
    arguments.summary.write_text(
        json.dumps(summary, ensure_ascii=False, indent=2, allow_nan=True)
        + "\n",
        encoding="utf-8",
    )
    run_id, registered = record_experiment(
        stage="H5",
        experiment_group="h5_robustness",
        run_name="h5_robustness_matrix",
        run_kind="deterministic_model",
        summary=summary,
        parameters={
            "target_laps_per_scenario": arguments.laps,
            "scenario_count": len(scenarios),
        },
        artifacts=[
            arguments.observations,
            arguments.scenarios,
            arguments.summary,
        ],
        source_summary=arguments.summary,
        attempt_label=arguments.attempt_label,
        parent_run_id=arguments.parent_run_id,
        change_reason_en=arguments.change_reason_en,
        change_reason_zh=arguments.change_reason_zh,
    )

    aggregate = summary["aggregate_metrics"]
    print(
        f"Scenarios / 场景数：{aggregate['scenario_count']}\n"
        f"Passed scenarios / 通过场景："
        f"{aggregate['passed_scenarios']}/"
        f"{aggregate['scenario_count']}\n"
        f"Completed laps / 完成圈数："
        f"{aggregate['completed_laps_total']}/"
        f"{aggregate['target_laps_total']}\n"
        f"Worst P95 cross-track error / 最差P95横向误差："
        f"{aggregate['worst_p95_cross_track_error_m']:.3f} m\n"
        f"Acceptance passed / 验收通过："
        f"{summary['acceptance_passed']}\n"
        f"Experiment run / 实验运行：{run_id}\n"
        f"New registry entry / 新登记记录：{registered}"
    )
    if not summary["acceptance_passed"]:
        for scenario_summary in summary["scenario_summaries"]:
            if scenario_summary["acceptance_passed"]:
                continue
            failed = [
                name
                for name, passed in scenario_summary[
                    "acceptance_checks"
                ].items()
                if not passed
            ]
            print(
                f"Failed scenario / 未通过场景："
                f"{scenario_summary['scenario']['name']} -> "
                + ", ".join(failed)
            )
        raise SystemExit(2)
    print("H5_ROBUSTNESS_EVALUATION_OK / H5鲁棒性评估成功")


if __name__ == "__main__":
    main()
