"""Tests for explicit experiment CSV export."""

import csv
from pathlib import Path

import pytest

from adaptive_assist import JointParameters, JointState, JointTorques, OneDofJointModel
from adaptive_assist.controllers import ControllerOutput
from adaptive_assist.experiments import (
    CSV_COLUMNS,
    SCENARIO_SCHEMA_VERSION,
    ConstantReference,
    JointReference,
    ScenarioConfig,
    run_closed_loop_experiment,
    run_open_loop_experiment,
    write_experiment_csv,
)
from adaptive_assist.safety import (
    SafetyInterventionReason,
    SafetyLimits,
    SafetySupervisor,
)


class FixedController:
    """Return one deterministic requested assistive torque."""

    def __init__(self, requested_torque_n_m: float) -> None:
        self.requested_torque_n_m = requested_torque_n_m

    def compute(
        self,
        state: JointState,
        reference: JointReference,
    ) -> ControllerOutput:
        del state, reference
        return ControllerOutput(self.requested_torque_n_m)


def test_csv_contains_expected_headers_rows_and_values(tmp_path: Path) -> None:
    parameters = JointParameters(2.0, 0.0, 0.0, 9.81, 0.0, 0.0, 0.0)
    scenario = ScenarioConfig(
        schema_version=SCENARIO_SCHEMA_VERSION,
        scenario_name="csv_test",
        duration_s=0.2,
        time_step_s=0.1,
        initial_state=JointState(0.0, 0.0),
        joint_parameters=parameters,
        reference=ConstantReference(JointReference(0.25, 0.0, 0.0)),
        torques=JointTorques(assistive_torque_n_m=2.0),
    )
    result = run_open_loop_experiment(scenario, OneDofJointModel(parameters))
    output_path = tmp_path / "experiment.csv"

    returned_path = write_experiment_csv(result, output_path)

    assert returned_path == output_path
    assert output_path.is_file()
    with output_path.open(encoding="utf-8", newline="") as csv_file:
        rows = list(csv.DictReader(csv_file))
    assert len(rows) == len(result.samples)
    assert tuple(rows[0]) == CSV_COLUMNS
    assert float(rows[0]["time_s"]) == pytest.approx(0.0)
    assert float(rows[0]["reference_angle_rad"]) == pytest.approx(0.25)
    assert float(rows[0]["requested_assistive_torque_n_m"]) == pytest.approx(2.0)
    assert float(rows[0]["applied_assistive_torque_n_m"]) == pytest.approx(2.0)
    assert rows[0]["safety_supervision_active"] == "False"
    assert rows[0]["safety_intervened"] == "False"
    assert rows[0]["safety_intervention_reasons"] == ""
    assert float(rows[1]["actual_angle_rad"]) == pytest.approx(0.01)


def test_csv_records_requested_applied_and_intervention_data(tmp_path: Path) -> None:
    parameters = JointParameters(2.0, 0.0, 0.0, 9.81, 0.0, 0.0, 0.0)
    scenario = ScenarioConfig(
        schema_version=SCENARIO_SCHEMA_VERSION,
        scenario_name="supervised_csv_test",
        duration_s=0.1,
        time_step_s=0.1,
        initial_state=JointState(0.0, 0.0),
        joint_parameters=parameters,
        reference=ConstantReference(JointReference(0.0, 0.0, 0.0)),
        torques=JointTorques(),
    )
    supervisor = SafetySupervisor(SafetyLimits(2.0, -1.0, 1.0, 2.0, 0.0))
    result = run_closed_loop_experiment(
        scenario,
        OneDofJointModel(parameters),
        FixedController(4.0),
        supervisor,
    )
    output_path = tmp_path / "supervised.csv"

    write_experiment_csv(result, output_path)

    with output_path.open(encoding="utf-8", newline="") as csv_file:
        rows = list(csv.DictReader(csv_file))
    assert float(rows[0]["requested_assistive_torque_n_m"]) == pytest.approx(4.0)
    assert float(rows[0]["applied_assistive_torque_n_m"]) == pytest.approx(2.0)
    assert rows[0]["safety_supervision_active"] == "True"
    assert rows[0]["safety_intervened"] == "True"
    assert rows[0]["safety_intervention_reasons"] == (
        SafetyInterventionReason.TORQUE_LIMIT.value
    )
