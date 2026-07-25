"""Deterministic scalar dynamics for a rotational assistive joint."""

import math
from dataclasses import dataclass


def _require_finite(name: str, value: float) -> None:
    """Raise a useful error when a model input is not finite."""
    if not math.isfinite(value):
        raise ValueError(f"{name} must be finite; received {value!r}")


@dataclass(frozen=True, slots=True)
class JointParameters:
    """Constant physical parameters for the joint model, expressed in SI units.

    Attributes:
        inertia_kg_m2: Equivalent rotational inertia in kilograms metres squared.
        mass_kg: Equivalent distal mass in kilograms.
        center_of_mass_distance_m: Distance from the joint to the centre of mass
            in metres.
        gravitational_acceleration_m_s2: Gravitational acceleration in metres per
            second squared.
        damping_n_m_s_per_rad: Viscous damping in newton metres seconds per radian.
        stiffness_n_m_per_rad: Passive stiffness in newton metres per radian.
        rest_angle_rad: Passive equilibrium angle in radians.
    """

    inertia_kg_m2: float
    mass_kg: float
    center_of_mass_distance_m: float
    gravitational_acceleration_m_s2: float
    damping_n_m_s_per_rad: float
    stiffness_n_m_per_rad: float
    rest_angle_rad: float

    def __post_init__(self) -> None:
        """Validate that every parameter is finite and physically admissible."""
        values = (
            ("inertia_kg_m2", self.inertia_kg_m2),
            ("mass_kg", self.mass_kg),
            ("center_of_mass_distance_m", self.center_of_mass_distance_m),
            (
                "gravitational_acceleration_m_s2",
                self.gravitational_acceleration_m_s2,
            ),
            ("damping_n_m_s_per_rad", self.damping_n_m_s_per_rad),
            ("stiffness_n_m_per_rad", self.stiffness_n_m_per_rad),
            ("rest_angle_rad", self.rest_angle_rad),
        )
        for name, value in values:
            _require_finite(name, value)

        if self.inertia_kg_m2 <= 0.0:
            raise ValueError("inertia_kg_m2 must be greater than zero")
        if self.mass_kg < 0.0:
            raise ValueError("mass_kg must be greater than or equal to zero")
        if self.center_of_mass_distance_m < 0.0:
            raise ValueError(
                "center_of_mass_distance_m must be greater than or equal to zero"
            )
        if self.gravitational_acceleration_m_s2 <= 0.0:
            raise ValueError(
                "gravitational_acceleration_m_s2 must be greater than zero"
            )
        if self.damping_n_m_s_per_rad < 0.0:
            raise ValueError(
                "damping_n_m_s_per_rad must be greater than or equal to zero"
            )
        if self.stiffness_n_m_per_rad < 0.0:
            raise ValueError(
                "stiffness_n_m_per_rad must be greater than or equal to zero"
            )


@dataclass(frozen=True, slots=True)
class JointState:
    """Joint angle and angular velocity in SI units."""

    angle_rad: float
    angular_velocity_rad_s: float

    def __post_init__(self) -> None:
        """Reject non-finite states before evaluating the dynamics."""
        _require_finite("angle_rad", self.angle_rad)
        _require_finite("angular_velocity_rad_s", self.angular_velocity_rad_s)


@dataclass(frozen=True, slots=True)
class JointTorques:
    """Externally applied joint torques in newton metres."""

    human_torque_n_m: float = 0.0
    assistive_torque_n_m: float = 0.0
    disturbance_torque_n_m: float = 0.0

    def __post_init__(self) -> None:
        """Reject non-finite torque inputs before evaluating the dynamics."""
        _require_finite("human_torque_n_m", self.human_torque_n_m)
        _require_finite("assistive_torque_n_m", self.assistive_torque_n_m)
        _require_finite("disturbance_torque_n_m", self.disturbance_torque_n_m)


@dataclass(frozen=True, slots=True)
class OneDofJointModel:
    """Simulator-independent deterministic model of one rotational joint."""

    parameters: JointParameters

    def gravity_torque_n_m(self, angle_rad: float) -> float:
        """Return the signed gravitational torque term in newton metres.

        Positive angles produce positive gravitational torque under the model's
        sign convention. This term is subtracted in the equation of motion.
        """
        _require_finite("angle_rad", angle_rad)
        parameters = self.parameters
        return (
            parameters.mass_kg
            * parameters.gravitational_acceleration_m_s2
            * parameters.center_of_mass_distance_m
            * math.sin(angle_rad)
        )

    def passive_torque_n_m(self, state: JointState) -> float:
        """Return the signed damping-plus-stiffness torque in newton metres.

        This term is subtracted in the equation of motion, so positive damping
        opposes motion and positive stiffness acts toward the passive rest angle.
        """
        parameters = self.parameters
        damping_torque_n_m = (
            parameters.damping_n_m_s_per_rad * state.angular_velocity_rad_s
        )
        stiffness_torque_n_m = parameters.stiffness_n_m_per_rad * (
            state.angle_rad - parameters.rest_angle_rad
        )
        return damping_torque_n_m + stiffness_torque_n_m

    def total_applied_torque_n_m(self, torques: JointTorques) -> float:
        """Return the sum of human, assistive, and disturbance torques."""
        return math.fsum(
            (
                torques.human_torque_n_m,
                torques.assistive_torque_n_m,
                torques.disturbance_torque_n_m,
            )
        )

    def angular_acceleration_rad_s2(
        self,
        state: JointState,
        torques: JointTorques,
    ) -> float:
        """Return angular acceleration in radians per second squared."""
        net_torque_n_m = (
            self.total_applied_torque_n_m(torques)
            - self.gravity_torque_n_m(state.angle_rad)
            - self.passive_torque_n_m(state)
        )
        return net_torque_n_m / self.parameters.inertia_kg_m2

    def step(
        self,
        state: JointState,
        torques: JointTorques,
        time_step_s: float,
    ) -> JointState:
        """Advance the state by one fixed semi-implicit Euler step.

        The acceleration is evaluated at the input state. Angular velocity is
        updated first, and angle is then updated using the new velocity. The
        immutable input state is never modified.
        """
        _require_finite("time_step_s", time_step_s)
        if time_step_s <= 0.0:
            raise ValueError("time_step_s must be greater than zero")

        acceleration_rad_s2 = self.angular_acceleration_rad_s2(state, torques)
        next_angular_velocity_rad_s = (
            state.angular_velocity_rad_s + acceleration_rad_s2 * time_step_s
        )
        next_angle_rad = state.angle_rad + next_angular_velocity_rad_s * time_step_s
        return JointState(
            angle_rad=next_angle_rad,
            angular_velocity_rad_s=next_angular_velocity_rad_s,
        )
