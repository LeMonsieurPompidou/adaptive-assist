"""Unit tests for deterministic simulation safety supervision."""

import math
from dataclasses import replace

import pytest

from adaptive_assist import JointState
from adaptive_assist.safety import (
    SafetyInterventionReason,
    SafetyLimits,
    SafetySupervisor,
)


def _limits() -> SafetyLimits:
    return SafetyLimits(
        max_abs_assistive_torque_n_m=4.0,
        min_joint_angle_rad=-0.5,
        max_joint_angle_rad=0.5,
        max_abs_joint_velocity_rad_s=1.0,
        fallback_assistive_torque_n_m=0.25,
    )


def test_in_range_finite_torque_passes_unchanged() -> None:
    result = SafetySupervisor(_limits()).apply(JointState(0.0, 0.0), 2.0)

    assert result.requested_assistive_torque_n_m == pytest.approx(2.0)
    assert result.applied_assistive_torque_n_m == pytest.approx(2.0)
    assert not result.intervened
    assert result.intervention_reasons == ()


@pytest.mark.parametrize(
    ("requested_torque_n_m", "expected_torque_n_m"),
    [(8.0, 4.0), (-8.0, -4.0)],
)
def test_excessive_torque_is_clipped_symmetrically(
    requested_torque_n_m: float,
    expected_torque_n_m: float,
) -> None:
    result = SafetySupervisor(_limits()).apply(
        JointState(0.0, 0.0),
        requested_torque_n_m,
    )

    assert result.applied_assistive_torque_n_m == pytest.approx(expected_torque_n_m)
    assert result.intervention_reasons == (SafetyInterventionReason.TORQUE_LIMIT,)


@pytest.mark.parametrize("requested_torque_n_m", [4.0, -4.0])
def test_exact_torque_boundary_passes(requested_torque_n_m: float) -> None:
    result = SafetySupervisor(_limits()).apply(
        JointState(0.0, 0.0),
        requested_torque_n_m,
    )

    assert result.applied_assistive_torque_n_m == requested_torque_n_m
    assert not result.intervened


@pytest.mark.parametrize(
    "state",
    [
        JointState(-0.5, 0.0),
        JointState(0.5, 0.0),
        JointState(0.0, -1.0),
        JointState(0.0, 1.0),
    ],
)
def test_exact_state_boundaries_pass(state: JointState) -> None:
    result = SafetySupervisor(_limits()).apply(state, 2.0)

    assert result.applied_assistive_torque_n_m == pytest.approx(2.0)
    assert not result.intervened


@pytest.mark.parametrize("requested_torque_n_m", [math.nan, math.inf, -math.inf])
def test_non_finite_command_invokes_fallback(requested_torque_n_m: float) -> None:
    result = SafetySupervisor(_limits()).apply(
        JointState(0.0, 0.0),
        requested_torque_n_m,
    )

    assert result.applied_assistive_torque_n_m == pytest.approx(0.25)
    assert result.intervention_reasons == (
        SafetyInterventionReason.INVALID_REQUESTED_COMMAND,
    )


@pytest.mark.parametrize("angle_rad", [-0.5001, 0.5001])
def test_position_outside_range_invokes_fallback(angle_rad: float) -> None:
    result = SafetySupervisor(_limits()).apply(JointState(angle_rad, 0.0), 2.0)

    assert result.applied_assistive_torque_n_m == pytest.approx(0.25)
    assert result.intervention_reasons == (
        SafetyInterventionReason.JOINT_POSITION_LIMIT,
    )


@pytest.mark.parametrize("velocity_rad_s", [1.0001, -1.0001])
def test_velocity_outside_range_invokes_fallback(velocity_rad_s: float) -> None:
    result = SafetySupervisor(_limits()).apply(
        JointState(0.0, velocity_rad_s),
        2.0,
    )

    assert result.applied_assistive_torque_n_m == pytest.approx(0.25)
    assert result.intervention_reasons == (
        SafetyInterventionReason.JOINT_VELOCITY_LIMIT,
    )


def test_invalid_command_has_precedence_over_state_and_torque_limits() -> None:
    result = SafetySupervisor(_limits()).apply(
        JointState(0.6, 1.5),
        math.inf,
    )

    assert result.applied_assistive_torque_n_m == pytest.approx(0.25)
    assert result.intervention_reasons == (
        SafetyInterventionReason.INVALID_REQUESTED_COMMAND,
    )


def test_multiple_state_reasons_have_deterministic_order() -> None:
    result = SafetySupervisor(_limits()).apply(JointState(0.6, -1.5), 8.0)

    assert result.applied_assistive_torque_n_m == pytest.approx(0.25)
    assert result.intervention_reasons == (
        SafetyInterventionReason.JOINT_POSITION_LIMIT,
        SafetyInterventionReason.JOINT_VELOCITY_LIMIT,
    )


def test_identical_inputs_produce_identical_results() -> None:
    supervisor = SafetySupervisor(_limits())
    state = JointState(0.1, -0.2)

    assert supervisor.apply(state, 8.0) == supervisor.apply(state, 8.0)


def test_supervisor_does_not_mutate_state() -> None:
    supervisor = SafetySupervisor(_limits())
    state = JointState(0.1, -0.2)

    supervisor.apply(state, 8.0)

    assert state == JointState(0.1, -0.2)


@pytest.mark.parametrize(
    ("field", "value"),
    [
        ("max_abs_assistive_torque_n_m", -0.1),
        ("max_abs_joint_velocity_rad_s", -0.1),
        ("max_abs_assistive_torque_n_m", math.inf),
        ("min_joint_angle_rad", math.nan),
        ("max_joint_angle_rad", math.inf),
        ("max_abs_joint_velocity_rad_s", math.nan),
        ("fallback_assistive_torque_n_m", math.inf),
    ],
)
def test_invalid_limit_values_are_rejected(field: str, value: float) -> None:
    with pytest.raises(ValueError, match=field):
        replace(_limits(), **{field: value})


@pytest.mark.parametrize(
    ("min_angle_rad", "max_angle_rad"),
    [(0.5, 0.5), (0.6, 0.5)],
)
def test_invalid_angle_range_is_rejected(
    min_angle_rad: float,
    max_angle_rad: float,
) -> None:
    with pytest.raises(ValueError, match="min_joint_angle_rad"):
        replace(
            _limits(),
            min_joint_angle_rad=min_angle_rad,
            max_joint_angle_rad=max_angle_rad,
        )


def test_fallback_must_be_within_torque_limit() -> None:
    with pytest.raises(ValueError, match="fallback_assistive_torque_n_m"):
        replace(_limits(), fallback_assistive_torque_n_m=4.1)
