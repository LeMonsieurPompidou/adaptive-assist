"""Integration tests for supervision in the generic closed-loop runner."""

import math
from dataclasses import dataclass
from pathlib import Path
from unittest.mock import patch

import pytest

from adaptive_assist import (
    JointParameters,
    JointState,
    JointTorques,
    OneDofJointModel,
)
from adaptive_assist.controllers import (
    ComputedTorqueController,
    ControllerOutput,
    ImpedanceController,
    JointController,
    load_computed_torque_controller_parameters,
    load_impedance_controller_parameters,
)
from adaptive_assist.experiments import (
    SCENARIO_SCHEMA_VERSION,
    ConstantReference,
    JointReference,
    ScenarioConfig,
    load_scenario,
    run_closed_loop_experiment,
)
from adaptive_assist.safety import (
    SafetyInterventionReason,
    SafetyLimits,
    SafetySupervisor,
    load_safety_limits,
)

REPOSITORY_ROOT = Path(__file__).resolve().parents[1]


@dataclass(frozen=True, slots=True)
class FixedController:
    """Return one fixed requested torque through the controller protocol."""

    requested_torque_n_m: float

    def compute(
        self,
        state: JointState,
        reference: JointReference,
    ) -> ControllerOutput:
        del state, reference
        return ControllerOutput(self.requested_torque_n_m)


def _scenario() -> ScenarioConfig:
    return ScenarioConfig(
        schema_version=SCENARIO_SCHEMA_VERSION,
        scenario_name="safety_runner_test",
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


def _supervisor() -> SafetySupervisor:
    return SafetySupervisor(SafetyLimits(2.0, -1.0, 1.0, 2.0, 0.0))


def test_runner_records_requested_and_applied_torque_separately() -> None:
    scenario = _scenario()

    result = run_closed_loop_experiment(
        scenario,
        OneDofJointModel(scenario.joint_parameters),
        FixedController(8.0),
        _supervisor(),
    )

    assert result.metadata.safety_supervision_active
    for sample in result.samples:
        assert sample.requested_assistive_torque_n_m == pytest.approx(8.0)
        assert sample.applied_torques.assistive_torque_n_m == pytest.approx(2.0)
        assert sample.safety_intervened
        assert sample.safety_intervention_reasons == (
            SafetyInterventionReason.TORQUE_LIMIT,
        )


def test_plant_evolves_from_applied_not_requested_torque() -> None:
    scenario = _scenario()

    result = run_closed_loop_experiment(
        scenario,
        OneDofJointModel(scenario.joint_parameters),
        FixedController(8.0),
        _supervisor(),
    )

    assert result.samples[0].angular_acceleration_rad_s2 == pytest.approx(1.0)
    assert result.samples[-1].actual_state.angle_rad == pytest.approx(0.03)
    assert result.samples[-1].actual_state.angular_velocity_rad_s == pytest.approx(0.2)


def test_invalid_controller_request_reaches_supervisor_fallback() -> None:
    scenario = _scenario()

    result = run_closed_loop_experiment(
        scenario,
        OneDofJointModel(scenario.joint_parameters),
        FixedController(math.nan),
        _supervisor(),
    )

    first_sample = result.samples[0]
    assert math.isnan(first_sample.requested_assistive_torque_n_m)
    assert first_sample.applied_torques.assistive_torque_n_m == pytest.approx(0.0)
    assert first_sample.safety_intervention_reasons == (
        SafetyInterventionReason.INVALID_REQUESTED_COMMAND,
    )


def test_invalid_controller_request_without_supervisor_is_rejected() -> None:
    scenario = _scenario()

    with pytest.raises(ValueError, match="requires an active safety supervisor"):
        run_closed_loop_experiment(
            scenario,
            OneDofJointModel(scenario.joint_parameters),
            FixedController(math.inf),
        )


def test_external_torques_are_unchanged_by_supervision() -> None:
    scenario = _scenario()

    result = run_closed_loop_experiment(
        scenario,
        OneDofJointModel(scenario.joint_parameters),
        FixedController(8.0),
        _supervisor(),
    )

    assert all(
        sample.applied_torques.human_torque_n_m == 0.5
        and sample.applied_torques.disturbance_torque_n_m == -0.5
        for sample in result.samples
    )


def test_direct_pass_through_remains_explicit_and_deterministic() -> None:
    scenario = _scenario()
    model = OneDofJointModel(scenario.joint_parameters)

    first_result = run_closed_loop_experiment(
        scenario,
        model,
        FixedController(8.0),
    )
    second_result = run_closed_loop_experiment(
        scenario,
        model,
        FixedController(8.0),
    )

    assert first_result == second_result
    assert not first_result.metadata.safety_supervision_active
    assert all(
        sample.requested_assistive_torque_n_m
        == sample.applied_torques.assistive_torque_n_m
        == 8.0
        and not sample.safety_intervened
        for sample in first_result.samples
    )


def test_both_controllers_use_the_same_supervisor_interface() -> None:
    scenario = load_scenario(
        REPOSITORY_ROOT / "configs/scenarios/nominal_tracking.json"
    )
    supervisor = SafetySupervisor(
        load_safety_limits(REPOSITORY_ROOT / "configs/safety/nominal_limits.json")
    )
    controllers: tuple[JointController, ...] = (
        ImpedanceController(
            load_impedance_controller_parameters(
                REPOSITORY_ROOT / "configs/controllers/impedance_baseline.json"
            )
        ),
        ComputedTorqueController(
            parameters=load_computed_torque_controller_parameters(
                REPOSITORY_ROOT / "configs/controllers/computed_torque_baseline.json"
            ),
            nominal_model=OneDofJointModel(scenario.joint_parameters),
        ),
    )

    results = tuple(
        run_closed_loop_experiment(
            scenario,
            OneDofJointModel(scenario.joint_parameters),
            controller,
            supervisor,
        )
        for controller in controllers
    )

    assert all(result.metadata.safety_supervision_active for result in results)
    assert all(len(result.samples) == scenario.step_count + 1 for result in results)


def test_supervisor_is_evaluated_at_final_sample_without_extra_step() -> None:
    scenario = _scenario()
    model = OneDofJointModel(scenario.joint_parameters)
    supervisor = _supervisor()
    original_apply = SafetySupervisor.apply
    original_step = OneDofJointModel.step

    with (
        patch.object(
            SafetySupervisor,
            "apply",
            autospec=True,
            side_effect=original_apply,
        ) as apply_mock,
        patch.object(
            OneDofJointModel,
            "step",
            autospec=True,
            side_effect=original_step,
        ) as step_mock,
    ):
        result = run_closed_loop_experiment(
            scenario,
            model,
            FixedController(8.0),
            supervisor,
        )

    assert apply_mock.call_count == scenario.step_count + 1
    assert step_mock.call_count == scenario.step_count
    assert result.samples[-1].time_s == scenario.duration_s
