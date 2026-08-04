import csv
import json
import math
from datetime import datetime
from pathlib import Path

import moving_cone_detector as moving_detector
from bilingual_console import configure_utf8_console


configure_utf8_console()


# 公平测试参数 / Fair-test settings
TEST_SPEEDS_MPS = (1.0, 2.0, 4.0)
TARGET_DISTANCE_METERS = 30.0
REPETITIONS_PER_SPEED = 4
FIXED_DELTA_SECONDS = 0.05


def frames_for_distance(speed_mps):
    """计算走完固定距离需要的帧数。/ Calculate frames needed for a fixed distance."""
    return math.ceil(
        TARGET_DISTANCE_METERS
        / (speed_mps * FIXED_DELTA_SECONDS)
    )


def aggregate_trials(speed_mps, frames_per_trial, trials):
    """汇总同一速度的多次结果。/ Aggregate repeated trials at one speed."""
    total_tp = sum(item["true_positives"] for item in trials)
    total_fp = sum(item["false_positives"] for item in trials)
    total_fn = sum(item["false_negatives"] for item in trials)

    # 用总计数重新计算综合评分。/ Recalculate combined scores from total counts.
    precision = (
        total_tp / (total_tp + total_fp)
        if total_tp + total_fp > 0
        else 0.0
    )
    recall = (
        total_tp / (total_tp + total_fn)
        if total_tp + total_fn > 0
        else 0.0
    )
    f1_score = (
        2.0 * precision * recall / (precision + recall)
        if precision + recall > 0
        else 0.0
    )

    # 按正确匹配数量加权位置误差。/ Weight position error by correct matches.
    weighted_error_sum = sum(
        item["mean_position_error_m"] * item["true_positives"]
        for item in trials
        if item["mean_position_error_m"] is not None
    )
    mean_error = (
        weighted_error_sum / total_tp
        if total_tp > 0
        else None
    )
    valid_max_errors = [
        item["max_position_error_m"]
        for item in trials
        if item["max_position_error_m"] is not None
    ]

    return {
        "speed_mps": speed_mps,
        "repetitions": len(trials),
        "frames_per_trial": frames_per_trial,
        "total_simulation_frames": frames_per_trial * len(trials),
        "target_distance_per_trial_m": TARGET_DISTANCE_METERS,
        "evaluated_frames": sum(
            item["evaluated_frames"] for item in trials
        ),
        "true_positives": total_tp,
        "false_positives": total_fp,
        "false_negatives": total_fn,
        "precision": round(precision, 4),
        "recall": round(recall, 4),
        "f1_score": round(f1_score, 4),
        "mean_position_error_m": (
            round(mean_error, 4) if mean_error is not None else None
        ),
        "max_position_error_m": (
            round(max(valid_max_errors), 4)
            if valid_max_errors
            else None
        ),
    }


def write_summary_csv(path, speed_results):
    """保存便于 Excel 查看的一览表。/ Save a summary table for Excel."""
    fieldnames = [
        "speed_mps",
        "repetitions",
        "frames_per_trial",
        "total_simulation_frames",
        "target_distance_per_trial_m",
        "evaluated_frames",
        "true_positives",
        "false_positives",
        "false_negatives",
        "precision",
        "recall",
        "f1_score",
        "mean_position_error_m",
        "max_position_error_m",
    ]
    with path.open("w", newline="", encoding="utf-8-sig") as file:
        writer = csv.DictWriter(file, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(speed_results)


def main():
    """依次运行三种速度并生成总表。/ Run three speeds and build one report."""
    output_directory = Path("output")
    output_directory.mkdir(exist_ok=True)
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    csv_path = output_directory / f"multi_speed_summary_{timestamp}.csv"
    json_path = output_directory / f"multi_speed_summary_{timestamp}.json"

    all_trials = []
    speed_results = []

    print("Starting the fair multi-speed test. / 开始多速度公平测试。")
    print(
        f"Distance per speed / 每个速度距离：{TARGET_DISTANCE_METERS:.0f} m; "
        f"repetitions / 重复次数：{REPETITIONS_PER_SPEED}"
    )

    # 每个速度运行相同距离的独立试验。/ Run independent equal-distance trials at each speed.
    for speed_mps in TEST_SPEEDS_MPS:
        frames = frames_for_distance(speed_mps)
        trials_for_speed = []

        for repetition in range(1, REPETITIONS_PER_SPEED + 1):
            print(
                f"\nSpeed / 速度 {speed_mps:.1f} m/s: "
                f"trial / 试验 {repetition}/{REPETITIONS_PER_SPEED}, "
                f"{frames} frames / 帧"
            )
            run_result = moving_detector.main(
                vehicle_speed_mps=speed_mps,
                simulation_frames=frames,
                show_frame_output=False,
            )
            if run_result is None:
                raise RuntimeError("Test did not finish; check the vehicle spawn point. / 测试未完成，请检查车辆生成位置。")

            trial_summary = dict(run_result["summary"])
            trial_summary["repetition"] = repetition
            trial_summary["summary_path"] = run_result["summary_path"]
            trials_for_speed.append(trial_summary)
            all_trials.append(trial_summary)

        combined = aggregate_trials(
            speed_mps,
            frames,
            trials_for_speed,
        )
        speed_results.append(combined)
        print(
            f"Speed summary / 速度汇总 {speed_mps:.1f} m/s: "
            f"P={combined['precision']:.1%}, "
            f"R={combined['recall']:.1%}, "
            f"F1={combined['f1_score']:.1%}, "
            f"error / 误差={combined['mean_position_error_m']} m"
        )

    # 保存机器可读 JSON 和表格 CSV。/ Save machine-readable JSON and tabular CSV.
    report = {
        "test_design": {
            "speeds_mps": list(TEST_SPEEDS_MPS),
            "target_distance_per_trial_m": TARGET_DISTANCE_METERS,
            "repetitions_per_speed": REPETITIONS_PER_SPEED,
            "fixed_delta_seconds": FIXED_DELTA_SECONDS,
        },
        "speed_results": speed_results,
        "trials": all_trials,
    }
    json_path.write_text(
        json.dumps(report, ensure_ascii=False, indent=2),
        encoding="utf-8",
    )
    write_summary_csv(csv_path, speed_results)

    from tools.export_bilingual_reports import export_all

    export_all(output_directory, output_directory / "bilingual")
    print("\nMulti-speed test completed. / 多速度测试完成。")
    print(f"Summary CSV / 汇总CSV：{csv_path}")
    print(f"Complete JSON / 完整JSON：{json_path}")
    print("Bilingual report copies refreshed. / 双语报告副本已刷新。")


if __name__ == "__main__":
    main()
