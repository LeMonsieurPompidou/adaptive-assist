"""Tests for deterministic robustness-summary CSV export."""

import csv
from pathlib import Path

from adaptive_assist.controllers import (
    load_computed_torque_controller_parameters,
    load_impedance_controller_parameters,
)
from adaptive_assist.evaluation import (
    ROBUSTNESS_CSV_COLUMNS,
    RobustnessController,
    RobustnessSweepResult,
    load_robustness_sweep_config,
    run_robustness_sweep,
    write_robustness_summary_csv,
)
from adaptive_assist.experiments import load_scenario
from adaptive_assist.safety import load_safety_limits

REPOSITORY_ROOT = Path(__file__).resolve().parents[1]


def _sweep() -> RobustnessSweepResult:
    configuration = load_robustness_sweep_config(
        REPOSITORY_ROOT / "configs/robustness/model_mismatch_sweep.json"
    )
    scenario = load_scenario(REPOSITORY_ROOT / configuration.scenario_config_path)
    return run_robustness_sweep(
        configuration,
        scenario,
        load_impedance_controller_parameters(
            REPOSITORY_ROOT / configuration.impedance_controller_config_path
        ),
        load_computed_torque_controller_parameters(
            REPOSITORY_ROOT / configuration.computed_torque_controller_config_path
        ),
        load_safety_limits(REPOSITORY_ROOT / configuration.safety_limits_config_path),
    )


def test_robustness_csv_has_expected_headers_rows_and_values(tmp_path: Path) -> None:
    result = _sweep()
    output_path = tmp_path / "robustness.csv"

    returned_path = write_robustness_summary_csv(result, output_path)

    assert returned_path == output_path
    with output_path.open(encoding="utf-8", newline="") as csv_file:
        rows = list(csv.DictReader(csv_file))
    assert tuple(rows[0]) == ROBUSTNESS_CSV_COLUMNS
    assert len(rows) == len(result.runs) == 24
    assert rows[0]["controller"] == RobustnessController.IMPEDANCE.value
    assert rows[0]["case_name"] == "nominal"
    assert rows[0]["varied_parameter"] == "nominal"
    assert rows[0]["scale_factor"] == "1.0"
    assert rows[-1]["controller"] == RobustnessController.COMPUTED_TORQUE.value
    assert rows[-1]["case_name"] == "combined_moderate_mismatch"
    assert rows[-1]["varied_parameter"] == "combined"
    assert rows[-1]["scale_factor"] == ""


def test_repeated_csv_exports_have_identical_order_and_content(tmp_path: Path) -> None:
    result = _sweep()
    first_path = tmp_path / "first.csv"
    second_path = tmp_path / "second.csv"

    write_robustness_summary_csv(result, first_path)
    write_robustness_summary_csv(result, second_path)

    assert first_path.read_bytes() == second_path.read_bytes()
