"""Behavior tests for deterministic computed-torque control."""

import math

import pytest

from adaptive_assist import JointParameters, JointState, OneDofJointModel
from adaptive_assist.controllers import (
    ComputedTorqueController,
    ComputedTorqueControllerParameters,
)
from adaptive_assist.experiments import JointReference


def _model(
    *,
    inertia_kg_m2: float = 2.0,
    mass_kg: float = 0.0,
    center_of_mass_distance_m: float = 0.0,
    damping_n_m_s_per_rad: float = 0.0,
    stiffness_n_m_per_rad: float = 0.0,
    rest_angle_rad: float = 0.0,
) -> OneDofJointModel:
    return OneDofJointModel(
        JointParameters(
            inertia_kg_m2=inertia_kg_m2,
            mass_kg=mass_kg,
            center_of_mass_distance_m=center_of_mass_distance_m,
            gravitational_acceleration_m_s2=9.81,
            damping_n_m_s_per_rad=damping_n_m_s_per_rad,
            stiffness_n_m_per_rad=stiffness_n_m_per_rad,
            rest_angle_rad=rest_angle_rad,
        )
    )


def _controller(
    *,
    model: OneDofJointModel | None = None,
    proportional_gain_n_m_per_rad: float = 10.0,
    derivative_gain_n_m_s_per_rad: float = 2.0,
) -> ComputedTorqueController:
    return ComputedTorqueController(
        parameters=ComputedTorqueControllerParameters(
            proportional_gain_n_m_per_rad=proportional_gain_n_m_per_rad,
            derivative_gain_n_m_s_per_rad=derivative_gain_n_m_s_per_rad,
        ),
        nominal_model=_model() if model is None else model,
    )


def _requested_torque(
    controller: ComputedTorqueController,
    state: JointState,
    reference: JointReference,
) -> float:
    return controller.compute(state, reference).requested_assistive_torque_n_m


def test_zero_error_and_acceleration_produce_model_compensation() -> None:
    model = _model(
        mass_kg=2.0,
        center_of_mass_distance_m=0.4,
        damping_n_m_s_per_rad=0.5,
        stiffness_n_m_per_rad=3.0,
        rest_angle_rad=0.1,
    )
    state = JointState(0.3, 0.2)
    reference = JointReference(0.3, 0.2, 0.0)

    requested_torque = _requested_torque(_controller(model=model), state, reference)

    assert requested_torque == pytest.approx(
        model.gravity_torque_n_m(state.angle_rad) + model.passive_torque_n_m(state)
    )


def test_equilibrium_without_errors_or_acceleration_produces_zero_torque() -> None:
    model = _model(
        mass_kg=2.0,
        center_of_mass_distance_m=0.4,
        damping_n_m_s_per_rad=0.5,
        stiffness_n_m_per_rad=3.0,
        rest_angle_rad=0.0,
    )
    state = JointState(0.0, 0.0)

    requested_torque = _requested_torque(
        _controller(model=model),
        state,
        JointReference(0.0, 0.0, 0.0),
    )

    assert requested_torque == pytest.approx(0.0)


@pytest.mark.parametrize(
    ("reference_acceleration_rad_s2", "expected_torque_n_m"),
    [(1.5, 3.0), (-1.5, -3.0)],
)
def test_reference_acceleration_produces_inertial_feedforward(
    reference_acceleration_rad_s2: float,
    expected_torque_n_m: float,
) -> None:
    controller = _controller(
        proportional_gain_n_m_per_rad=0.0,
        derivative_gain_n_m_s_per_rad=0.0,
    )

    requested_torque = _requested_torque(
        controller,
        JointState(0.0, 0.0),
        JointReference(0.0, 0.0, reference_acceleration_rad_s2),
    )

    assert requested_torque == pytest.approx(expected_torque_n_m)


@pytest.mark.parametrize(
    ("actual_angle_rad", "reference_angle_rad", "expected_torque_n_m"),
    [(0.0, 0.2, 2.0), (0.2, 0.0, -2.0)],
)
def test_proportional_feedback_has_expected_sign(
    actual_angle_rad: float,
    reference_angle_rad: float,
    expected_torque_n_m: float,
) -> None:
    controller = _controller(derivative_gain_n_m_s_per_rad=0.0)

    requested_torque = _requested_torque(
        controller,
        JointState(actual_angle_rad, 0.0),
        JointReference(reference_angle_rad, 0.0, 0.0),
    )

    assert requested_torque == pytest.approx(expected_torque_n_m)


@pytest.mark.parametrize(
    ("actual_velocity_rad_s", "reference_velocity_rad_s", "expected_torque_n_m"),
    [(0.0, 0.5, 1.0), (0.5, 0.0, -1.0)],
)
def test_derivative_feedback_has_expected_sign(
    actual_velocity_rad_s: float,
    reference_velocity_rad_s: float,
    expected_torque_n_m: float,
) -> None:
    controller = _controller(proportional_gain_n_m_per_rad=0.0)

    requested_torque = _requested_torque(
        controller,
        JointState(0.0, actual_velocity_rad_s),
        JointReference(0.0, reference_velocity_rad_s, 0.0),
    )

    assert requested_torque == pytest.approx(expected_torque_n_m)


def test_gravity_compensation_uses_nominal_model_result() -> None:
    model = _model(mass_kg=2.0, center_of_mass_distance_m=0.4)
    state = JointState(math.pi / 6.0, 0.0)
    controller = _controller(
        model=model,
        proportional_gain_n_m_per_rad=0.0,
        derivative_gain_n_m_s_per_rad=0.0,
    )

    requested_torque = _requested_torque(
        controller,
        state,
        JointReference(state.angle_rad, 0.0, 0.0),
    )

    assert requested_torque == pytest.approx(model.gravity_torque_n_m(state.angle_rad))


def test_passive_compensation_uses_nominal_model_result() -> None:
    model = _model(
        damping_n_m_s_per_rad=0.5,
        stiffness_n_m_per_rad=3.0,
        rest_angle_rad=0.1,
    )
    state = JointState(0.3, -0.2)
    controller = _controller(
        model=model,
        proportional_gain_n_m_per_rad=0.0,
        derivative_gain_n_m_s_per_rad=0.0,
    )

    requested_torque = _requested_torque(
        controller,
        state,
        JointReference(state.angle_rad, state.angular_velocity_rad_s, 0.0),
    )

    assert requested_torque == pytest.approx(model.passive_torque_n_m(state))


def test_reference_acceleration_changes_computed_torque_output() -> None:
    controller = _controller()
    state = JointState(0.1, -0.2)

    without_acceleration = controller.compute(
        state,
        JointReference(0.3, 0.4, 0.0),
    )
    with_acceleration = controller.compute(
        state,
        JointReference(0.3, 0.4, 2.0),
    )

    assert with_acceleration.requested_assistive_torque_n_m == pytest.approx(
        without_acceleration.requested_assistive_torque_n_m + 4.0
    )


def test_identical_inputs_produce_identical_outputs() -> None:
    controller = _controller()
    state = JointState(0.1, -0.2)
    reference = JointReference(0.3, 0.4, -2.0)

    assert controller.compute(state, reference) == controller.compute(state, reference)


def test_compute_does_not_mutate_inputs() -> None:
    controller = _controller()
    state = JointState(0.1, -0.2)
    reference = JointReference(0.3, 0.4, -2.0)
    model = controller.nominal_model

    controller.compute(state, reference)

    assert state == JointState(0.1, -0.2)
    assert reference == JointReference(0.3, 0.4, -2.0)
    assert controller.nominal_model is model


@pytest.mark.parametrize(
    ("proportional_gain_n_m_per_rad", "derivative_gain_n_m_s_per_rad"),
    [(0.0, 4.0), (20.0, 0.0), (0.0, 0.0)],
)
def test_zero_gains_are_valid(
    proportional_gain_n_m_per_rad: float,
    derivative_gain_n_m_s_per_rad: float,
) -> None:
    parameters = ComputedTorqueControllerParameters(
        proportional_gain_n_m_per_rad=proportional_gain_n_m_per_rad,
        derivative_gain_n_m_s_per_rad=derivative_gain_n_m_s_per_rad,
    )

    assert parameters.proportional_gain_n_m_per_rad >= 0.0
    assert parameters.derivative_gain_n_m_s_per_rad >= 0.0


@pytest.mark.parametrize(
    ("field", "value"),
    [
        ("proportional_gain_n_m_per_rad", -0.1),
        ("derivative_gain_n_m_s_per_rad", -0.1),
    ],
)
def test_negative_gains_are_rejected(field: str, value: float) -> None:
    gains = {
        "proportional_gain_n_m_per_rad": 20.0,
        "derivative_gain_n_m_s_per_rad": 4.0,
    }
    gains[field] = value

    with pytest.raises(ValueError, match=field):
        ComputedTorqueControllerParameters(**gains)


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
        "proportional_gain_n_m_per_rad": 20.0,
        "derivative_gain_n_m_s_per_rad": 4.0,
    }
    gains[field] = value

    with pytest.raises(ValueError, match=field):
        ComputedTorqueControllerParameters(**gains)
