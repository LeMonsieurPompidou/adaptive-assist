"""Tests for deterministic controller-driven experiment execution."""

from dataclasses import dataclass, field
from unittest.mock import patch

import pytest

from adaptive_assist import (
    JointParameters,
    JointState,
    JointTorques,
    OneDofJointModel,
)
from adaptive_assist.controllers import ControllerOutput, JointController
from adaptive_assist.experiments import (
    SCENARIO_SCHEMA_VERSION,
    ConstantReference,
    JointReference,
    ScenarioConfig,
    run_closed_loop_experiment,
)


@dataclass
class RecordingController:
    """Test controller that records every state/reference pair it receives."""

    requested_torque_n_m: float
    calls: list[tuple[JointState, JointReference]] = field(default_factory=list)

    def compute(
        self,
        state: JointState,
        reference: JointReference,
    ) -> ControllerOutput:
        self.calls.append((state, reference))
        return ControllerOutput(self.requested_torque_n_m)


def _scenario() -> ScenarioConfig:
    return ScenarioConfig(
        schema_version=SCENARIO_SCHEMA_VERSION,
        scenario_name="closed_loop_test",
        duration_s=0.2,
        time_step_s=0.1,
        initial_state=JointState(0.0, 0.0),
        joint_parameters=JointParameters(2.0, 0.0, 0.0, 9.81, 0.0, 0.0, 0.0),
        reference=ConstantReference(JointReference(0.25, 0.0, 0.0)),
        torques=JointTorques(
            human_torque_n_m=0.5,
            assistive_torque_n_m=99.0,
            disturbance_torque_n_m=-0.5,
        ),
    )


def test_controller_is_evaluated_once_per_sample_at_recorded_state() -> None:
    scenario = _scenario()
    controller = RecordingController(2.0)

    result = run_closed_loop_experiment(
        scenario,
        OneDofJointModel(scenario.joint_parameters),
        controller,
    )

    assert len(controller.calls) == len(result.samples) == scenario.step_count + 1
    assert controller.calls == [
        (sample.actual_state, sample.reference) for sample in result.samples
    ]
    assert result.samples[0].time_s == 0.0
    assert result.samples[-1].time_s == scenario.duration_s


def test_requested_torque_is_applied_with_configured_external_torques() -> None:
    scenario = _scenario()

    result = run_closed_loop_experiment(
        scenario,
        OneDofJointModel(scenario.joint_parameters),
        RecordingController(2.0),
    )

    for sample in result.samples:
        assert sample.applied_torques == JointTorques(
            human_torque_n_m=0.5,
            assistive_torque_n_m=2.0,
            disturbance_torque_n_m=-0.5,
        )


def test_closed_loop_runner_uses_exactly_one_step_between_samples() -> None:
    scenario = _scenario()
    model = OneDofJointModel(scenario.joint_parameters)
    original_step = OneDofJointModel.step

    with patch.object(
        OneDofJointModel,
        "step",
        autospec=True,
        side_effect=original_step,
    ) as step_mock:
        result = run_closed_loop_experiment(
            scenario,
            model,
            RecordingController(2.0),
        )

    assert step_mock.call_count == scenario.step_count
    assert result.samples[-1].actual_state.angle_rad == pytest.approx(0.03)
    assert result.samples[-1].actual_state.angular_velocity_rad_s == pytest.approx(0.2)


def test_closed_loop_runs_are_identical() -> None:
    scenario = _scenario()
    model = OneDofJointModel(scenario.joint_parameters)

    first_result = run_closed_loop_experiment(
        scenario,
        model,
        RecordingController(2.0),
    )
    second_result = run_closed_loop_experiment(
        scenario,
        model,
        RecordingController(2.0),
    )

    assert first_result == second_result


def test_closed_loop_runner_does_not_mutate_inputs() -> None:
    scenario = _scenario()
    model = OneDofJointModel(scenario.joint_parameters)
    initial_state = scenario.initial_state
    configured_torques = scenario.torques

    run_closed_loop_experiment(scenario, model, RecordingController(2.0))

    assert scenario.initial_state is initial_state
    assert scenario.torques is configured_torques
    assert model.parameters is scenario.joint_parameters


def test_closed_loop_accepts_joint_controller_protocol() -> None:
    controller: JointController = RecordingController(0.0)
    scenario = _scenario()

    result = run_closed_loop_experiment(
        scenario,
        OneDofJointModel(scenario.joint_parameters),
        controller,
    )

    assert result.samples[0].applied_torques.assistive_torque_n_m == pytest.approx(0.0)
