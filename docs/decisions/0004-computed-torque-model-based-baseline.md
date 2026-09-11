# ADR 0004: Computed-Torque Model-Based Baseline

- **Status:** Accepted
- **Date:** 2026-09-11

## Context

After establishing impedance feedback, the project needs an interpretable
model-based baseline that exercises desired acceleration and known nominal
dynamics without introducing optimization, constraints, or new dependencies.
The design should demonstrate that the existing `JointController` interface and
closed-loop runner support multiple controllers unchanged.

## Decision

Implement computed-torque control as nominal inertial feedforward, current-state
gravity compensation, current-state passive-torque compensation, and explicit
proportional-derivative tracking feedback.

The controller owns a separate immutable `OneDofJointModel` as its nominal
model. It reads nominal inertia and reuses the model's public gravity and passive
torque methods rather than repeating dynamics equations. The experiment plant
remains a distinct model object.

Use the same initial feedback gains as the impedance baseline: `Kp = 20 N
m/rad` and `Kd = 4 N m s/rad`. Store those gains in a strict,
version-controlled JSON file separate from scenario dynamics.

Return an unsaturated requested torque through the existing `ControllerOutput`.
Safety supervision and applied-command constraints remain outside both the
controller and plant.

## Rationale

- Computed torque is algebraic, deterministic, and easier to inspect than MPC.
- Reusing `OneDofJointModel` keeps the plant equations under one ownership
  boundary.
- Separate nominal and actual model objects enable later parameter-mismatch
  experiments without changing interfaces.
- Equal feedback gains make the initial comparison focus on model compensation
  and inertial feedforward rather than opportunistic tuning.
- Standard-library JSON preserves the dependency-free runtime.
- The existing generic runner proves reusable without controller-specific
  branching.

## Nominal-model assumptions

The nominal comparison assumes controller and plant inertia, mass,
centre-of-mass distance, gravity, damping, stiffness, and rest angle match. It
also assumes exact state and reference derivatives. Human and disturbance
torques are external and are not canceled by the controller.

Mismatch in any modeled parameter can make compensation incomplete or
incorrect. This sensitivity is expected and will motivate later robustness and
residual-learning evaluation; it is not addressed in this milestone.

## Consequences

- The project has a second deterministic classical baseline using the existing
  controller and experiment contracts.
- Reference acceleration now influences requested torque through nominal
  inertia.
- Nominal gravity and passive dynamics are compensated without equation
  duplication.
- A matching nominal simulation may track differently from impedance control,
  but that result is not evidence of global superiority or real-world accuracy.
- Requested torque is unbounded and directly applied until a safety supervisor
  exists.

## Alternatives considered

- **Model predictive control:** deferred because it would require optimizer,
  horizon, objective, and constraint decisions beyond the current milestone.
- **Copy plant equations into the controller:** rejected because duplicated
  dynamics would drift and obscure ownership.
- **Share the actual plant object with the controller:** not required; separate
  immutable objects make nominal-versus-actual ownership explicit.
- **Retune feedback gains:** deferred because matching the impedance gains makes
  the first comparison easier to interpret.
- **Add saturation or limits:** rejected because final-command constraints
  belong to the future safety supervisor.

## Reconsider when

- robustness experiments define parameter-mismatch suites;
- state estimation changes the controller input contract;
- actuator dynamics invalidate direct torque application;
- a safety supervisor records requested and applied commands separately;
- evidence justifies retuning under a documented procedure; or
- MPC requirements, constraints, and computational budgets are defined.
