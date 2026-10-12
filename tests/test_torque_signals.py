"""Tests for deterministic torque signals and runner integration."""

import math
from dataclasses import FrozenInstanceError, dataclass, field, replace

import pytest

from adaptive_assist import JointParameters, JointState, JointTorques, OneDofJointModel
from adaptive_assist.controllers import ControllerOutput
from adaptive_assist.experiments import (
    SCENARIO_SCHEMA_VERSION,
    ConstantReference,
    ConstantTorqueSignal,
    JointReference,
    PulseTorqueSignal,
    ScenarioConfig,
    run_closed_loop_experiment,
)
from adaptive_assist.safety import (
    SafetyInterventionReason,
    SafetyLimits,
    SafetySupervisor,
)


@dataclass(frozen=True)
class ConstantController:
    requested_torque_n_m: float = 0.0

    def compute(self, state: JointState, reference: JointReference) -> ControllerOutput:
        del state, reference
        return ControllerOutput(self.requested_torque_n_m)


@dataclass
class RecordingController:
    calls: list[tuple[JointState, JointReference]] = field(default_factory=list)

    def compute(self, state: JointState, reference: JointReference) -> ControllerOutput:
        self.calls.append((state, reference))
        return ControllerOutput(0.0)


def _scenario() -> ScenarioConfig:
    return ScenarioConfig(
        schema_version=SCENARIO_SCHEMA_VERSION,
        scenario_name="time_varying_disturbance_test",
        duration_s=0.3,
        time_step_s=0.1,
        initial_state=JointState(0.0, 0.0),
        joint_parameters=JointParameters(1.0, 0.0, 0.0, 9.81, 0.0, 0.0, 0.0),
        reference=ConstantReference(JointReference(0.0, 0.0, 0.0)),
        torques=JointTorques(disturbance_torque_n_m=0.25),
    )


def test_constant_torque_signal_is_constant() -> None:
    signal = ConstantTorqueSignal(-0.75)

    assert signal.evaluate(0.0) == pytest.approx(-0.75)
    assert signal.evaluate(10.0) == pytest.approx(-0.75)


def test_non_finite_constant_torque_is_rejected() -> None:
    with pytest.raises(ValueError, match="torque_n_m"):
        ConstantTorqueSignal(math.inf)


def test_pulse_uses_half_open_interval_boundaries() -> None:
    signal = PulseTorqueSignal(1.0, start_time_s=0.5, duration_s=0.25)

    assert signal.evaluate(0.49) == 0.0
    assert signal.evaluate(0.5) == pytest.approx(1.0)
    assert signal.evaluate(0.6) == pytest.approx(1.0)
    assert signal.evaluate(0.75) == 0.0
    assert signal.evaluate(0.8) == 0.0


def test_negative_pulse_preserves_sign() -> None:
    signal = PulseTorqueSignal(-1.0, start_time_s=0.5, duration_s=0.25)

    assert signal.evaluate(0.6) == pytest.approx(-1.0)


def test_pulse_end_is_exclusive_despite_decimal_roundoff() -> None:
    signal = PulseTorqueSignal(1.0, start_time_s=0.1, duration_s=0.2)

    assert signal.evaluate(0.3) == 0.0


@pytest.mark.parametrize(
    ("pulse_torque_n_m", "start_time_s", "duration_s"),
    [
        (math.inf, 0.0, 1.0),
        (1.0, math.nan, 1.0),
        (1.0, -0.1, 1.0),
        (1.0, 0.0, 0.0),
        (1.0, 0.0, math.inf),
    ],
)
def test_invalid_pulse_values_are_rejected(
    pulse_torque_n_m: float,
    start_time_s: float,
    duration_s: float,
) -> None:
    with pytest.raises(ValueError):
        PulseTorqueSignal(pulse_torque_n_m, start_time_s, duration_s)


@pytest.mark.parametrize("time_s", [-0.1, math.nan, math.inf])
def test_invalid_evaluation_time_is_rejected(time_s: float) -> None:
    with pytest.raises(ValueError):
        ConstantTorqueSignal(0.0).evaluate(time_s)
    with pytest.raises(ValueError):
        PulseTorqueSignal(1.0, 0.0, 1.0).evaluate(time_s)


def test_torque_signals_are_immutable() -> None:
    signal = PulseTorqueSignal(1.0, 0.5, 0.25)

    with pytest.raises(FrozenInstanceError):
        signal.pulse_torque_n_m = 2.0  # type: ignore[misc]


def test_time_varying_disturbance_reaches_joint_torques() -> None:
    scenario = _scenario()
    result = run_closed_loop_experiment(
        scenario,
        OneDofJointModel(scenario.joint_parameters),
        ConstantController(),
        disturbance_signal=PulseTorqueSignal(1.0, 0.1, 0.1),
    )

    assert tuple(
        sample.applied_torques.disturbance_torque_n_m for sample in result.samples
    ) == pytest.approx((0.0, 1.0, 0.0, 0.0))
    assert result.samples[2].actual_state.angular_velocity_rad_s == pytest.approx(0.1)


def test_controller_remains_unaware_of_disturbance_signal() -> None:
    scenario = _scenario()
    controller = RecordingController()
    result = run_closed_loop_experiment(
        scenario,
        OneDofJointModel(scenario.joint_parameters),
        controller,
        disturbance_signal=PulseTorqueSignal(1.0, 0.1, 0.1),
    )

    assert controller.calls == [
        (sample.actual_state, sample.reference) for sample in result.samples
    ]


def test_supervisor_still_constrains_only_assistive_torque() -> None:
    scenario = _scenario()
    supervisor = SafetySupervisor(
        SafetyLimits(
            max_abs_assistive_torque_n_m=2.0,
            min_joint_angle_rad=-1.0,
            max_joint_angle_rad=1.0,
            max_abs_joint_velocity_rad_s=10.0,
            fallback_assistive_torque_n_m=0.0,
        )
    )
    result = run_closed_loop_experiment(
        scenario,
        OneDofJointModel(scenario.joint_parameters),
        ConstantController(3.0),
        supervisor,
        disturbance_signal=PulseTorqueSignal(1.0, 0.1, 0.1),
    )

    pulse_sample = result.samples[1]
    assert pulse_sample.requested_assistive_torque_n_m == pytest.approx(3.0)
    assert pulse_sample.applied_torques.assistive_torque_n_m == pytest.approx(2.0)
    assert pulse_sample.applied_torques.disturbance_torque_n_m == pytest.approx(1.0)
    assert pulse_sample.safety_intervention_reasons == (
        SafetyInterventionReason.TORQUE_LIMIT,
    )


def test_existing_constant_disturbance_behavior_is_unchanged() -> None:
    scenario = _scenario()
    model = OneDofJointModel(scenario.joint_parameters)

    legacy_result = run_closed_loop_experiment(scenario, model, ConstantController(0.5))
    signal_result = run_closed_loop_experiment(
        scenario,
        model,
        ConstantController(0.5),
        disturbance_signal=ConstantTorqueSignal(
            scenario.torques.disturbance_torque_n_m
        ),
    )

    assert signal_result == legacy_result
    assert scenario == replace(scenario)
