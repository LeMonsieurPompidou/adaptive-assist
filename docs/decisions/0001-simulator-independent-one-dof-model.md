# ADR 0001: Simulator-independent one-DOF plant model

- **Status:** Accepted
- **Date:** 2026-07-25

## Context

The project needs a first executable plant model before classical and learned
controllers can be designed or compared. Introducing a full robotics simulator
at this stage would combine basic equation validation with simulator selection,
dependency management, and adapter design. Those concerns have different
requirements and should be evaluated separately.

The initial model must be easy to inspect, deterministic under repeated inputs,
and sufficient to test the sign, units, passive behavior, and integration of a
single assistive joint abstraction.

## Decision

Implement the first plant as a simulator-independent scalar Python model with:

- one rotational degree of freedom;
- standard-library mathematics and immutable typed dataclasses;
- explicit SI-unit field names and validated parameters and inputs;
- deterministic fixed-step execution; and
- semi-implicit Euler integration.

The plant exposes gravity torque, passive torque, applied torque, acceleration,
and state stepping without controller, safety-policy, logging, or simulator
framework dependencies.

## Reasons

- **Simulator independence:** keeps the governing physics testable without an
  external engine and establishes a reference for future simulator adapters.
- **Standard library:** avoids a major dependency before vectorized workloads or
  simulator-specific requirements exist.
- **Determinism:** makes regression tests and later controller comparisons easier
  to reproduce and diagnose.
- **One rotational degree of freedom:** keeps sign conventions, units, and energy
  paths understandable for the first milestone.
- **Semi-implicit Euler:** is simple and deterministic, updates position using
  the new velocity, and generally behaves better for basic mechanical systems
  than explicit Euler at the same small fixed step.

## Consequences

Positive consequences:

- the model can be tested quickly with no runtime dependencies;
- physics and integration behavior are explicit and reviewable;
- future controllers can depend on a small typed plant-facing API; and
- the implementation can serve as a reference when validating another
  simulator.

Trade-offs and limitations:

- the scalar model does not provide contacts, constraints, advanced integrators,
  visualization, or high-performance batched simulation;
- one degree of freedom cannot represent multi-joint coupling or whole-limb
  biomechanics;
- numerical accuracy depends on the caller's fixed time step; and
- future simulator integration will require an adapter and cross-validation.

## Alternatives considered

- **MuJoCo or another physics simulator immediately:** deferred because simulator
  selection requires a separate evaluation and is unnecessary for this scalar
  milestone.
- **NumPy or SciPy implementation:** not selected because scalar standard-library
  arithmetic is sufficient and avoids an unneeded runtime dependency.
- **Explicit Euler:** not selected because it updates position using the old
  velocity and has less favorable behavior for basic mechanical systems.
- **Runge-Kutta integration:** not selected because its additional evaluations
  and complexity are not justified for the first deterministic reference model.
- **Multiple degrees of freedom:** deferred until the single-joint equations,
  interfaces, and evaluation needs are established.

## Future reconsideration criteria

Revisit this decision when the project needs contact dynamics, coupled joints,
high-throughput training, richer actuator or sensor models, real-time hardware
timing, or evidence that semi-implicit Euler is insufficient at the required
step size. Selecting MuJoCo or any other simulator remains a later ADR; this
decision neither selects nor excludes one.

