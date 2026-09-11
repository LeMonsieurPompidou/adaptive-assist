"""Behavior tests for the deterministic impedance controller."""

import math

import pytest

from adaptive_assist import JointState
from adaptive_assist.controllers import (
    ControllerOutput,
    ImpedanceController,
    ImpedanceControllerParameters,
)
from adaptive_assist.experiments import JointReference


def _controller(
    *,
    proportional_gain_n_m_per_rad: float = 10.0,
    derivative_gain_n_m_s_per_rad: float = 2.0,
) -> ImpedanceController:
    return ImpedanceController(
        ImpedanceControllerParameters(
            proportional_gain_n_m_per_rad=proportional_gain_n_m_per_rad,
            derivative_gain_n_m_s_per_rad=derivative_gain_n_m_s_per_rad,
        )
    )


def _compute(
    controller: ImpedanceController,
    *,
    actual_angle_rad: float = 0.0,
    actual_velocity_rad_s: float = 0.0,
    reference_angle_rad: float = 0.0,
    reference_velocity_rad_s: float = 0.0,
    reference_acceleration_rad_s2: float = 0.0,
) -> float:
    return controller.compute(
        JointState(actual_angle_rad, actual_velocity_rad_s),
        JointReference(
            reference_angle_rad,
            reference_velocity_rad_s,
            reference_acceleration_rad_s2,
        ),
    ).requested_assistive_torque_n_m


def test_zero_tracking_error_produces_zero_requested_torque() -> None:
    torque = _compute(
        _controller(),
        actual_angle_rad=0.3,
        actual_velocity_rad_s=-0.2,
        reference_angle_rad=0.3,
        reference_velocity_rad_s=-0.2,
    )

    assert torque == pytest.approx(0.0)


@pytest.mark.parametrize(
    ("actual_angle_rad", "reference_angle_rad", "expected_torque_n_m"),
    [(0.0, 0.2, 2.0), (0.2, 0.0, -2.0)],
)
def test_position_error_has_expected_torque_sign(
    actual_angle_rad: float,
    reference_angle_rad: float,
    expected_torque_n_m: float,
) -> None:
    torque = _compute(
        _controller(derivative_gain_n_m_s_per_rad=0.0),
        actual_angle_rad=actual_angle_rad,
        reference_angle_rad=reference_angle_rad,
    )

    assert torque == pytest.approx(expected_torque_n_m)


@pytest.mark.parametrize(
    ("actual_velocity_rad_s", "reference_velocity_rad_s", "expected_torque_n_m"),
    [(0.0, 0.5, 1.0), (0.5, 0.0, -1.0)],
)
def test_velocity_error_has_expected_derivative_contribution(
    actual_velocity_rad_s: float,
    reference_velocity_rad_s: float,
    expected_torque_n_m: float,
) -> None:
    torque = _compute(
        _controller(proportional_gain_n_m_per_rad=0.0),
        actual_velocity_rad_s=actual_velocity_rad_s,
        reference_velocity_rad_s=reference_velocity_rad_s,
    )

    assert torque == pytest.approx(expected_torque_n_m)


def test_proportional_and_derivative_terms_combine() -> None:
    torque = _compute(
        _controller(),
        actual_angle_rad=0.1,
        actual_velocity_rad_s=-0.2,
        reference_angle_rad=0.4,
        reference_velocity_rad_s=0.3,
    )

    assert torque == pytest.approx(4.0)


@pytest.mark.parametrize(
    ("proportional_gain_n_m_per_rad", "derivative_gain_n_m_s_per_rad", "expected"),
    [(0.0, 2.0, 1.0), (10.0, 0.0, 3.0), (0.0, 0.0, 0.0)],
)
def test_zero_gains_are_valid(
    proportional_gain_n_m_per_rad: float,
    derivative_gain_n_m_s_per_rad: float,
    expected: float,
) -> None:
    torque = _compute(
        _controller(
            proportional_gain_n_m_per_rad=proportional_gain_n_m_per_rad,
            derivative_gain_n_m_s_per_rad=derivative_gain_n_m_s_per_rad,
        ),
        reference_angle_rad=0.3,
        reference_velocity_rad_s=0.5,
    )

    assert torque == pytest.approx(expected)


@pytest.mark.parametrize(
    ("field", "value"),
    [
        ("proportional_gain_n_m_per_rad", math.inf),
        ("proportional_gain_n_m_per_rad", math.nan),
        ("derivative_gain_n_m_s_per_rad", -math.inf),
        ("derivative_gain_n_m_s_per_rad", math.nan),
    ],
)
def test_non_finite_gains_are_rejected(field: str, value: float) -> None:
    gains = {
        "proportional_gain_n_m_per_rad": 10.0,
        "derivative_gain_n_m_s_per_rad": 2.0,
    }
    gains[field] = value

    with pytest.raises(ValueError, match=field):
        ImpedanceControllerParameters(**gains)


@pytest.mark.parametrize(
    ("field", "value"),
    [
        ("proportional_gain_n_m_per_rad", -0.1),
        ("derivative_gain_n_m_s_per_rad", -0.1),
    ],
)
def test_negative_gains_are_rejected(field: str, value: float) -> None:
    gains = {
        "proportional_gain_n_m_per_rad": 10.0,
        "derivative_gain_n_m_s_per_rad": 2.0,
    }
    gains[field] = value

    with pytest.raises(ValueError, match=field):
        ImpedanceControllerParameters(**gains)


def test_identical_inputs_produce_identical_outputs() -> None:
    controller = _controller()
    state = JointState(0.1, -0.2)
    reference = JointReference(0.3, 0.4, -5.0)

    assert controller.compute(state, reference) == controller.compute(state, reference)


def test_compute_does_not_mutate_state_or_reference() -> None:
    controller = _controller()
    state = JointState(0.1, -0.2)
    reference = JointReference(0.3, 0.4, -5.0)

    controller.compute(state, reference)

    assert state == JointState(0.1, -0.2)
    assert reference == JointReference(0.3, 0.4, -5.0)


def test_reference_acceleration_does_not_affect_output() -> None:
    controller = _controller()
    state = JointState(0.1, -0.2)
    without_acceleration = JointReference(0.3, 0.4, 0.0)
    with_acceleration = JointReference(0.3, 0.4, 100.0)

    assert controller.compute(state, without_acceleration) == controller.compute(
        state,
        with_acceleration,
    )


def test_controller_output_preserves_non_finite_torque_for_supervision() -> None:
    output = ControllerOutput(math.inf)

    assert math.isinf(output.requested_assistive_torque_n_m)
