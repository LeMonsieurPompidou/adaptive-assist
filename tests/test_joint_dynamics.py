"""Physical-behavior tests for the deterministic one-DOF joint model."""

import math

import pytest

from adaptive_assist import (
    JointParameters,
    JointState,
    JointTorques,
    OneDofJointModel,
)


def _parameters(
    *,
    inertia_kg_m2: float = 1.0,
    mass_kg: float = 0.0,
    center_of_mass_distance_m: float = 0.0,
    gravitational_acceleration_m_s2: float = 9.81,
    damping_n_m_s_per_rad: float = 0.0,
    stiffness_n_m_per_rad: float = 0.0,
    rest_angle_rad: float = 0.0,
) -> JointParameters:
    return JointParameters(
        inertia_kg_m2=inertia_kg_m2,
        mass_kg=mass_kg,
        center_of_mass_distance_m=center_of_mass_distance_m,
        gravitational_acceleration_m_s2=gravitational_acceleration_m_s2,
        damping_n_m_s_per_rad=damping_n_m_s_per_rad,
        stiffness_n_m_per_rad=stiffness_n_m_per_rad,
        rest_angle_rad=rest_angle_rad,
    )


def test_zero_state_and_torques_produce_zero_acceleration() -> None:
    model = OneDofJointModel(_parameters())

    acceleration = model.angular_acceleration_rad_s2(
        JointState(angle_rad=0.0, angular_velocity_rad_s=0.0),
        JointTorques(),
    )

    assert acceleration == pytest.approx(0.0)


def test_positive_assistive_torque_produces_positive_acceleration() -> None:
    model = OneDofJointModel(_parameters(inertia_kg_m2=2.0))

    acceleration = model.angular_acceleration_rad_s2(
        JointState(angle_rad=0.0, angular_velocity_rad_s=0.0),
        JointTorques(assistive_torque_n_m=3.0),
    )

    assert acceleration == pytest.approx(1.5)


def test_gravity_torque_has_the_angle_sign() -> None:
    model = OneDofJointModel(_parameters(mass_kg=2.0, center_of_mass_distance_m=0.4))

    positive_torque = model.gravity_torque_n_m(math.pi / 6.0)
    negative_torque = model.gravity_torque_n_m(-math.pi / 6.0)

    assert positive_torque > 0.0
    assert negative_torque < 0.0
    assert negative_torque == pytest.approx(-positive_torque)


@pytest.mark.parametrize(
    ("angular_velocity_rad_s", "expected_acceleration_sign"),
    [(2.0, -1), (-2.0, 1)],
)
def test_damping_opposes_velocity(
    angular_velocity_rad_s: float,
    expected_acceleration_sign: int,
) -> None:
    model = OneDofJointModel(_parameters(damping_n_m_s_per_rad=0.5))

    acceleration = model.angular_acceleration_rad_s2(
        JointState(
            angle_rad=0.0,
            angular_velocity_rad_s=angular_velocity_rad_s,
        ),
        JointTorques(),
    )

    assert math.copysign(1.0, acceleration) == expected_acceleration_sign


@pytest.mark.parametrize(
    ("angle_rad", "expected_acceleration_sign"),
    [(0.5, -1), (-0.1, 1)],
)
def test_stiffness_acts_toward_rest_angle(
    angle_rad: float,
    expected_acceleration_sign: int,
) -> None:
    model = OneDofJointModel(_parameters(stiffness_n_m_per_rad=4.0, rest_angle_rad=0.2))

    acceleration = model.angular_acceleration_rad_s2(
        JointState(angle_rad=angle_rad, angular_velocity_rad_s=0.0),
        JointTorques(),
    )

    assert math.copysign(1.0, acceleration) == expected_acceleration_sign


def test_applied_torques_are_combined_before_dividing_by_inertia() -> None:
    model = OneDofJointModel(_parameters(inertia_kg_m2=2.0))
    torques = JointTorques(
        human_torque_n_m=1.0,
        assistive_torque_n_m=2.0,
        disturbance_torque_n_m=-0.5,
    )
    state = JointState(angle_rad=0.0, angular_velocity_rad_s=0.0)

    assert model.total_applied_torque_n_m(torques) == pytest.approx(2.5)
    assert model.angular_acceleration_rad_s2(state, torques) == pytest.approx(1.25)


def test_increasing_inertia_reduces_acceleration() -> None:
    state = JointState(angle_rad=0.0, angular_velocity_rad_s=0.0)
    torques = JointTorques(assistive_torque_n_m=2.0)
    lower_inertia_model = OneDofJointModel(_parameters(inertia_kg_m2=1.0))
    higher_inertia_model = OneDofJointModel(_parameters(inertia_kg_m2=4.0))

    lower_inertia_acceleration = lower_inertia_model.angular_acceleration_rad_s2(
        state, torques
    )
    higher_inertia_acceleration = higher_inertia_model.angular_acceleration_rad_s2(
        state, torques
    )

    assert higher_inertia_acceleration < lower_inertia_acceleration
    assert higher_inertia_acceleration == pytest.approx(
        lower_inertia_acceleration / 4.0
    )


def test_step_uses_semi_implicit_euler() -> None:
    model = OneDofJointModel(_parameters(inertia_kg_m2=2.0))
    initial_state = JointState(angle_rad=0.0, angular_velocity_rad_s=1.0)
    torques = JointTorques(assistive_torque_n_m=2.0)

    next_state = model.step(initial_state, torques, time_step_s=0.1)

    assert next_state.angular_velocity_rad_s == pytest.approx(1.1)
    assert next_state.angle_rad == pytest.approx(0.11)


def test_identical_runs_produce_identical_states() -> None:
    model = OneDofJointModel(
        _parameters(
            inertia_kg_m2=1.2,
            mass_kg=2.0,
            center_of_mass_distance_m=0.3,
            damping_n_m_s_per_rad=0.2,
            stiffness_n_m_per_rad=1.5,
            rest_angle_rad=0.1,
        )
    )
    torques = JointTorques(assistive_torque_n_m=0.8)

    def run() -> JointState:
        state = JointState(angle_rad=0.2, angular_velocity_rad_s=-0.1)
        for _ in range(100):
            state = model.step(state, torques, time_step_s=0.005)
        return state

    assert run() == run()


def test_invalid_parameter_ranges_are_rejected() -> None:
    with pytest.raises(ValueError, match="inertia_kg_m2"):
        _parameters(inertia_kg_m2=0.0)
    with pytest.raises(ValueError, match="inertia_kg_m2"):
        _parameters(inertia_kg_m2=-1.0)
    with pytest.raises(ValueError, match="mass_kg"):
        _parameters(mass_kg=-1.0)
    with pytest.raises(ValueError, match="center_of_mass_distance_m"):
        _parameters(center_of_mass_distance_m=-1.0)
    with pytest.raises(ValueError, match="gravitational_acceleration_m_s2"):
        _parameters(gravitational_acceleration_m_s2=0.0)
    with pytest.raises(ValueError, match="damping_n_m_s_per_rad"):
        _parameters(damping_n_m_s_per_rad=-1.0)
    with pytest.raises(ValueError, match="stiffness_n_m_per_rad"):
        _parameters(stiffness_n_m_per_rad=-1.0)


@pytest.mark.parametrize("non_finite_value", [math.inf, -math.inf, math.nan])
def test_non_finite_parameters_are_rejected(non_finite_value: float) -> None:
    with pytest.raises(ValueError, match="rest_angle_rad must be finite"):
        _parameters(rest_angle_rad=non_finite_value)


@pytest.mark.parametrize("time_step_s", [0.0, -0.01, math.inf, math.nan])
def test_invalid_time_steps_are_rejected(time_step_s: float) -> None:
    model = OneDofJointModel(_parameters())

    with pytest.raises(ValueError, match="time_step_s"):
        model.step(
            JointState(angle_rad=0.0, angular_velocity_rad_s=0.0),
            JointTorques(),
            time_step_s=time_step_s,
        )


def test_step_does_not_mutate_input_state() -> None:
    model = OneDofJointModel(_parameters())
    initial_state = JointState(angle_rad=0.25, angular_velocity_rad_s=-0.5)
    initial_values = (initial_state.angle_rad, initial_state.angular_velocity_rad_s)

    next_state = model.step(
        initial_state,
        JointTorques(assistive_torque_n_m=1.0),
        time_step_s=0.01,
    )

    assert initial_state.angle_rad == initial_values[0]
    assert initial_state.angular_velocity_rad_s == initial_values[1]
    assert next_state is not initial_state


def test_non_finite_states_and_torques_are_rejected() -> None:
    with pytest.raises(ValueError, match="angle_rad"):
        JointState(angle_rad=math.nan, angular_velocity_rad_s=0.0)
    with pytest.raises(ValueError, match="assistive_torque_n_m"):
        JointTorques(assistive_torque_n_m=math.inf)
