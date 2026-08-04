"""Evaluate selected H5 CARLA/ROS closed-track runs. / 评估选定的H5 CARLA/ROS闭环运行。"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import re
import sys

from evaluate_closed_loop_lap import evaluate, parse_log, write_csv
from experiment_registry import record_experiment


FAULT_PATTERN = re.compile(
    r"frame / 帧=(?P<frame>\d+).*?"
    r"fault_input / 故障输入=(?P<input>\d+).*?"
    r"fault_dropped / 故障漏检=(?P<dropped>\d+)"
)

if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
if hasattr(sys.stderr, "reconfigure"):
    sys.stderr.reconfigure(encoding="utf-8", errors="replace")


def fault_counts(path: Path) -> tuple[int, int]:
    """Sum periodic candidate-fault accounting. / 汇总周期性候选故障统计。"""
    input_count = 0
    dropped_count = 0
    for line in path.read_text(
        encoding="utf-8",
        errors="replace",
    ).splitlines():
        match = FAULT_PATTERN.search(line)
        if match is None:
            continue
        input_count += int(match.group("input"))
        dropped_count += int(match.group("dropped"))
    return input_count, dropped_count


def main() -> None:
    """Evaluate one selected H5 CARLA integration run. / 评估一个选定的H5 CARLA集成运行。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--log", type=Path, required=True)
    parser.add_argument("--csv", type=Path, required=True)
    parser.add_argument("--json", type=Path, required=True)
    parser.add_argument("--scenario", required=True)
    parser.add_argument("--target-laps", type=int, required=True)
    parser.add_argument("--dropout-rate", type=float, default=0.0)
    parser.add_argument("--noise-std-m", type=float, default=0.0)
    parser.add_argument(
        "--localization-drift-per-m",
        type=float,
        default=0.015,
    )
    parser.add_argument("--frames", type=int, default=0)
    parser.add_argument("--fault-seed", type=int, default=0)
    parser.add_argument("--attempt-label", default="automatic")
    parser.add_argument("--parent-run-id", default="")
    parser.add_argument(
        "--change-reason-en",
        default="Automated selected live CARLA H5 integration",
    )
    parser.add_argument(
        "--change-reason-zh",
        default="自动运行选定实时CARLA H5集成",
    )
    arguments = parser.parse_args()

    rows = parse_log(arguments.log)
    summary = evaluate(rows)
    metrics = summary["metrics"]
    criteria = summary["acceptance_criteria"]
    checks = summary["acceptance_checks"]
    input_count, dropped_count = fault_counts(arguments.log)
    retention_ratio = (
        (input_count - dropped_count) / input_count
        if input_count
        else 0.0
    )
    expected_retention = max(
        0.50,
        1.0 - float(arguments.dropout_rate) - 0.15,
    )

    # Raise the H4 one-lap threshold to the H5 run target and add fault
    # accounting checks. / 把H4单圈阈值提高到H5目标圈数，并增加故障统计检查。
    criteria["minimum_completed_laps"] = int(arguments.target_laps)
    criteria["minimum_fault_input_count"] = 20
    criteria["minimum_fault_retention_ratio"] = expected_retention
    checks["minimum_completed_laps"] = int(metrics["completed_laps"]) >= int(
        arguments.target_laps
    )
    checks["minimum_fault_input_count"] = input_count >= 20
    checks["minimum_fault_retention_ratio"] = (
        retention_ratio >= expected_retention
    )
    metrics["fault_input_count"] = input_count
    metrics["fault_dropped_count"] = dropped_count
    metrics["fault_retention_ratio"] = retention_ratio
    summary.update(
        {
            "stage": "H5 selected CARLA/ROS integration validation",
            "scenario": arguments.scenario,
            "target_laps": int(arguments.target_laps),
            "cone_dropout_rate": float(arguments.dropout_rate),
            "cone_noise_standard_deviation_m": float(
                arguments.noise_std_m
            ),
            "acceptance_passed": all(checks.values()),
            "note": (
                "This selected integration run uses live CARLA virtual "
                "LiDAR, cone clustering, tracking, map refinement, "
                "incremental relocalization, planning, and control."
            ),
        }
    )

    write_csv(rows, arguments.csv)
    arguments.json.parent.mkdir(parents=True, exist_ok=True)
    arguments.json.write_text(
        json.dumps(summary, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    run_id, registered = record_experiment(
        stage="H5",
        experiment_group="h5_live_carla",
        run_name=arguments.json.stem.replace("_evaluation", ""),
        run_kind="live_carla",
        summary=summary,
        parameters={
            "scenario": arguments.scenario,
            "target_laps": arguments.target_laps,
            "dropout_rate": arguments.dropout_rate,
            "noise_standard_deviation_m": arguments.noise_std_m,
            "localization_drift_per_m": (
                arguments.localization_drift_per_m
            ),
            "frames": arguments.frames,
            "fault_seed": arguments.fault_seed,
            "fixed_delta_seconds": 0.05,
        },
        artifacts=[arguments.log, arguments.csv, arguments.json],
        source_summary=arguments.json,
        attempt_label=arguments.attempt_label,
        parent_run_id=arguments.parent_run_id,
        change_reason_en=arguments.change_reason_en,
        change_reason_zh=arguments.change_reason_zh,
    )
    print(
        f"Scenario / 场景：{arguments.scenario}\n"
        f"Completed laps / 完成圈数：{metrics['completed_laps']}/"
        f"{arguments.target_laps}\n"
        f"Mean cross-track error / 平均横向误差："
        f"{metrics['mean_cross_track_error_m']:.3f} m\n"
        f"Fault retention / 故障后保留率：{retention_ratio:.3f}\n"
        f"Acceptance passed / 验收通过：{summary['acceptance_passed']}\n"
        f"Experiment run / 实验运行：{run_id}\n"
        f"New registry entry / 新登记记录：{registered}"
    )
    if not summary["acceptance_passed"]:
        failed = [name for name, passed in checks.items() if not passed]
        print("Failed checks / 未通过检查：" + ", ".join(failed))
        raise SystemExit(2)
    print(
        "H5_CARLA_INTEGRATION_EVALUATION_OK / "
        "H5 CARLA集成评估成功"
    )


if __name__ == "__main__":
    main()
