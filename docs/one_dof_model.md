# Deterministic One-DOF Joint Model

## Status and purpose

The repository implements a simulator-independent mathematical model of one
rotational assistive joint. It is intended for deterministic engineering tests
and future controller comparisons. It is not a clinically accurate human-joint
model, a controller, or evidence that a physical system is safe.

The public API is available from both `adaptive_assist` and
`adaptive_assist.dynamics`.

## Governing equation

The rotational equation of motion is

```text
J q_ddot = tau_human + tau_assist + tau_disturbance
           - tau_gravity - tau_passive
```

where

```text
tau_gravity = m g l sin(q)
tau_passive = b q_dot + k (q - q_rest)
```

and therefore

```text
q_ddot = (tau_human + tau_assist + tau_disturbance
          - m g l sin(q) - b q_dot - k (q - q_rest)) / J
```

## Variables and SI units

| Symbol | API name | Meaning | SI unit |
| --- | --- | --- | --- |
| `q` | `angle_rad` | Joint angle | rad |
| `q_dot` | `angular_velocity_rad_s` | Joint angular velocity | rad/s |
| `q_ddot` | `angular_acceleration_rad_s2` | Joint angular acceleration | rad/s² |
| `J` | `inertia_kg_m2` | Equivalent rotational inertia | kg·m² |
| `m` | `mass_kg` | Equivalent distal mass | kg |
| `l` | `center_of_mass_distance_m` | Joint-to-centre-of-mass distance | m |
| `g` | `gravitational_acceleration_m_s2` | Gravitational acceleration | m/s² |
| `b` | `damping_n_m_s_per_rad` | Viscous damping coefficient | N·m·s/rad |
| `k` | `stiffness_n_m_per_rad` | Passive stiffness | N·m/rad |
| `q_rest` | `rest_angle_rad` | Passive equilibrium angle | rad |
| `tau_*` | `*_torque_n_m` | Applied or modelled torque | N·m |

## Sign convention

A positive applied torque increases `angle_rad`. Human, assistive, and
disturbance torques use this same sign convention and are summed before the
opposing terms are applied.

The zero angle is the gravitational equilibrium represented by `sin(q) = 0`.
For an angle between zero and π radians, the gravity expression is positive and
is subtracted from the applied torque. Viscous damping opposes angular velocity.
Passive stiffness acts toward `rest_angle_rad`.

## Model API

- `JointParameters` stores validated, constant physical parameters.
- `JointState` stores angle and angular velocity.
- `JointTorques` stores human, assistive, and disturbance torque inputs.
- `OneDofJointModel` calculates gravity torque, passive torque, total applied
  torque, angular acceleration, and one integration step.

All dataclasses are immutable. Parameters, states, torques, and time steps must
be finite. Inertia, gravitational acceleration, and the integration time step
must be positive. Mass, centre-of-mass distance, damping, and stiffness must be
non-negative. Invalid values raise `ValueError`; values are never clamped.

## Numerical integration

`OneDofJointModel.step` uses fixed-step semi-implicit Euler integration:

1. evaluate acceleration at the current state;
2. update angular velocity with that acceleration; and
3. update angle using the new angular velocity.

The caller supplies a finite time step greater than zero. The method returns a
new `JointState` and does not mutate its input state. Repeated runs with the same
parameters, inputs, initial state, and time step are deterministic.

## Assumptions

- Motion is limited to one rigid rotational degree of freedom.
- Inertia and all passive parameters remain constant during a run.
- Gravity follows a lumped point-mass term `m g l sin(q)`.
- Passive behavior is linear viscous damping plus linear stiffness.
- Applied torques remain constant within each integration step.
- State variables and torque inputs are available without sensor noise, delay,
  estimation error, or transport effects.
- Numerical behavior is evaluated with scalar Python floating-point arithmetic.

## Known limitations

The model does not include joint or actuator limits, torque saturation,
friction nonlinearities, backlash, compliance beyond the linear passive term,
muscle activation dynamics, contact, impacts, sensor dynamics, or parameter
identification. It does not implement a controller, safety supervisor, learned
policy, external simulator adapter, ROS 2 interface, or hardware interface.

Joint and actuator command constraints belong to an independent safety layer so
the plant remains free of controller and policy decisions. The implemented
simulation supervisor therefore sits outside this model; advanced constraints
remain future work. The current model must not be interpreted as a clinically
accurate human joint or used for human testing.

