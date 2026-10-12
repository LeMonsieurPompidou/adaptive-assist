"""Tests for deterministic excitation-summary CSV export."""

import csv
from pathlib import Path

from adaptive_assist.controllers import (
    load_computed_torque_controller_parameters,
    load_impedance_controller_parameters,
)
from adaptive_assist.evaluation import (
    EXCITATION_CSV_COLUMNS,
    DisturbanceProfile,
    ExcitationSweepResult,
    RobustnessController,
    load_excitation_sweep_config,
    run_excitation_sweep,
    write_excitation_summary_csv,
)
from adaptive_assist.experiments import load_scenario
from adaptive_assist.safety import load_safety_limits

REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
CONFIG_PATH = REPOSITORY_ROOT / "configs/robustness/excitation_disturbance_sweep.json"


def _sweep() -> ExcitationSweepResult:
    configuration = load_excitation_sweep_config(CONFIG_PATH)
    scenario = load_scenario(REPOSITORY_ROOT / configuration.scenario_config_path)
    return run_excitation_sweep(
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


def test_excitation_csv_has_expected_headers_order_and_values(tmp_path: Path) -> None:
    result = _sweep()
    output_path = tmp_path / "excitation.csv"

    returned_path = write_excitation_summary_csv(result, output_path)

    assert returned_path == output_path
    with output_path.open(encoding="utf-8", newline="") as csv_file:
        rows = list(csv.DictReader(csv_file))
    assert tuple(rows[0]) == EXCITATION_CSV_COLUMNS
    assert len(rows) == 72
    assert rows[0]["controller"] == RobustnessController.IMPEDANCE.value
    assert rows[0]["frequency_hz"] == "0.25"
    assert rows[0]["inertia_scale"] == "0.8"
    assert rows[0]["disturbance_profile"] == DisturbanceProfile.NONE.value
    assert rows[0]["recovery_time_s"] == ""
    assert rows[0]["recovered"] == ""
    assert rows[-1]["controller"] == RobustnessController.COMPUTED_TORQUE.value
    assert rows[-1]["disturbance_profile"] == (DisturbanceProfile.NEGATIVE_PULSE.value)
    assert rows[-1]["nominal_controller_inertia_kg_m2"] == "1.2"


def test_repeated_excitation_csv_exports_are_identical(tmp_path: Path) -> None:
    result = _sweep()
    first = tmp_path / "first.csv"
    second = tmp_path / "second.csv"

    write_excitation_summary_csv(result, first)
    write_excitation_summary_csv(result, second)

    assert first.read_bytes() == second.read_bytes()
