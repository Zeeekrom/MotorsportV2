"""Tests for the deterministic H5 benchmark. / 确定性H5基准测试。"""

import pytest

from motorsport_lidar_bridge.h5_benchmark import (
    ScenarioConfig,
    default_scenarios,
    evaluate_matrix,
    run_scenario,
)


def test_nominal_circle_completes_one_lap():
    """The integrated modules complete one nominal lap. / 集成模块可完成一圈正常场景。"""
    rows, summary = run_scenario(
        ScenarioConfig(
            name="unit_circle",
            track_shape="circle",
            target_laps=1,
            maximum_time_s=80.0,
            seed=901,
        )
    )

    assert rows
    assert summary["acceptance_passed"]
    assert summary["metrics"]["completed_laps"] == 1
    assert summary["metrics"]["p95_cross_track_error_m"] < 1.5


def test_dynamic_obstacle_stops_and_resumes():
    """The vehicle stops for and resumes after a temporary obstacle. / 车辆会为临时障碍停车并在其消失后恢复。"""
    _, summary = run_scenario(
        ScenarioConfig(
            name="unit_obstacle",
            track_shape="oval",
            target_laps=1,
            dynamic_obstacle=True,
            maximum_time_s=100.0,
            seed=902,
        )
    )

    assert summary["acceptance_passed"]
    assert summary["metrics"]["stopped_duration_s"] >= 0.5
    assert summary["metrics"]["resumed_after_obstacle"]
    assert summary["metrics"]["minimum_obstacle_distance_m"] >= 2.0


def test_default_matrix_covers_h5_scope():
    """The standard matrix covers every required H5 dimension. / 标准矩阵覆盖全部H5维度。"""
    scenarios = default_scenarios(target_laps=1)
    shapes = {scenario.track_shape for scenario in scenarios}

    assert shapes == {"circle", "oval", "technical"}
    assert any(scenario.dynamic_obstacle for scenario in scenarios)
    assert any(scenario.dropout_rate >= 0.35 for scenario in scenarios)
    assert any(
        scenario.noise_standard_deviation_m >= 0.18
        for scenario in scenarios
    )


def test_empty_matrix_is_rejected():
    """A matrix without scenarios is invalid. / 没有场景的矩阵无效。"""
    with pytest.raises(ValueError):
        evaluate_matrix([])
