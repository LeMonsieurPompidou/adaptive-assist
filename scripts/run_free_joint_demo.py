"""Run a short deterministic demonstration of the one-DOF joint model."""

from adaptive_assist import (
    JointParameters,
    JointState,
    JointTorques,
    OneDofJointModel,
)


def main() -> int:
    """Simulate a constant assistive torque and print the resulting state."""
    model = OneDofJointModel(
        JointParameters(
            inertia_kg_m2=1.2,
            mass_kg=2.5,
            center_of_mass_distance_m=0.25,
            gravitational_acceleration_m_s2=9.81,
            damping_n_m_s_per_rad=0.3,
            stiffness_n_m_per_rad=1.0,
            rest_angle_rad=0.0,
        )
    )
    state = JointState(angle_rad=0.0, angular_velocity_rad_s=0.0)
    torques = JointTorques(assistive_torque_n_m=1.0)
    time_step_s = 0.05
    duration_s = 0.5
    step_count = round(duration_s / time_step_s)

    print("Deterministic mathematical 1-DOF joint demonstration")
    print("This is not a controller benchmark; no controller is present.")
    print("Constant assistive torque: 1.000 N m")
    print(" time_s | angle_rad | velocity_rad_s | acceleration_rad_s2")
    print("--------+-----------+----------------+--------------------")

    for step_index in range(step_count + 1):
        time_s = step_index * time_step_s
        acceleration_rad_s2 = model.angular_acceleration_rad_s2(state, torques)
        print(
            f" {time_s:6.2f} | {state.angle_rad:9.5f} | "
            f"{state.angular_velocity_rad_s:14.5f} | "
            f"{acceleration_rad_s2:18.5f}"
        )
        if step_index < step_count:
            state = model.step(state, torques, time_step_s)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
