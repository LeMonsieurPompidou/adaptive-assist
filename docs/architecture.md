# Planned Architecture

This document describes the intended system decomposition. The deterministic
one-degree-of-freedom mathematical plant, deterministic open-loop experiment
framework, impedance controller, and foundation tooling listed under "Currently
implemented" are working software. State estimation, model-based control,
safety supervision, learning, broader experiment tracking, external simulation,
and hardware integration remain planned.

## Implemented open-loop flow

```mermaid
flowchart LR
    A[JSON scenario] --> B[ScenarioConfig]
    B --> C[ReferenceSignal]
    B --> D[Configured constant torques]
    B --> E[1-DOF mathematical plant]
    C --> F[ExperimentSample]
    D --> E
    E --> F
    F --> G[ExperimentResult]
    G --> H[CSV export]
    G --> I[RMSE and peak torque metrics]
```

This implemented path is deterministic and open loop. It contains no controller
or safety supervisor.

## Implemented impedance closed-loop flow

```mermaid
flowchart LR
    A[JSON scenario] --> B[ReferenceSignal]
    B --> C[JointReference]
    D[Current JointState] --> E[ImpedanceController]
    C --> E
    E --> F[ControllerOutput<br/>requested assistive torque]
    F --> G[Direct pass-through<br/>no safety supervisor]
    A --> H[Human and disturbance torques]
    G --> I[JointTorques<br/>applied plant inputs]
    H --> I
    I --> J[1-DOF mathematical plant]
    D --> J
    J --> K[ExperimentSample]
    C --> K
    I --> K
    J --> D
    K --> L[ExperimentResult]
    L --> M[CSV and metrics]
```

The controller request currently passes directly to the plant and is recorded
as applied assistive torque. This is an explicit temporary equality, not safety
supervision or a safety claim.

## Planned full system flow

```mermaid
flowchart LR
    A[Sensors or simulation state] --> B[State estimation]
    B --> C[Baseline controller]
    C --> D[Optional residual learned policy]
    D --> E[Safety supervisor]
    E --> F[Simulated or physical<br/>one-degree-of-freedom plant]
    F --> A

    A --> G[Experiment logger<br/>and evaluation]
    B --> G
    C --> G
    D --> G
    E --> G
    F --> G
```

The optional residual policy is intended to augment the baseline command. A
pass-through mode will allow classical controllers to use the same downstream
safety and evaluation path without a learned correction. The safety supervisor
owns the final command boundary; the learned policy does not bypass it.

## Implemented and planned modules

### Plant and sensing

The implemented scalar plant provides validated one-degree-of-freedom dynamics,
human, assistive, and disturbance torque inputs, and deterministic
semi-implicit Euler integration. It has no controller, joint-limit, or actuator
saturation decisions. Its API and limitations are documented in
[the model specification](one_dof_model.md).

External simulator and future physical-plant adapters remain planned. They may
expose a compatible plant-facing interface for comparison with the scalar
reference model. Planned sensor adapters will provide timestamped state data
without exposing hardware details to controllers.

### State estimation

State estimation will validate and transform measurements into a consistent
controller state. It may later include filtering, derivative estimation,
interaction-torque estimation, and explicit handling of stale or invalid data.

### Baseline controllers

The implemented `JointController` interface maps state and reference to a typed
requested assistive torque. `ImpedanceController` implements proportional
position-error and derivative velocity-error feedback with version-controlled
gains. It contains no saturation or safety logic. A model-based controller
remains planned and can later implement the same small interface.

### Residual learned policy

The optional learned policy will request a bounded residual torque based on a
defined observation. It will not directly actuate the plant. Training and
inference concerns will remain separate from the deterministic baseline and
safety paths.

### Safety supervisor

The supervisor will combine or filter requested torque, enforce configured
limits, detect invalid states and commands, select fallback behavior, and emit
structured intervention records. Its constraints must apply consistently in
training, simulation evaluation, and any future bench evaluation.

### Logging and evaluation

The implemented experiment layer records actual state, reference, applied
torques, plant acceleration, scenario identity, and deterministic execution
metadata for open- and closed-loop runs. It provides explicit standard-library
CSV export plus tracking RMSE and peak assistive torque outside controller
logic.

State estimates, controller components, safety events, richer provenance,
additional metrics, and an experiment-tracking service remain planned.

## Currently implemented

The repository currently contains only:

- the typed `adaptive_assist` package and status-reporting module entry point;
- the validated deterministic 1-DOF mathematical plant and integration step;
- physical-behavior tests and a constant-torque mathematical demonstration;
- strict versioned JSON scenarios and analytic reference signals;
- deterministic open-loop execution, immutable records, optional CSV export,
  and initial controller-independent metrics;
- an open-loop experiment demonstration;
- a deterministic impedance controller, strict gain configuration, closed-loop
  runner, engineering demonstration, and open-loop comparison;
- packaging and development-tool configuration;
- a standard-library environment checker;
- continuous integration; and
- model, scope, architecture, and decision-record documentation.

There is no sensor interface, estimator, model-based controller, learned policy,
safety supervisor, external simulator integration, ROS 2 integration, hardware
interface, or experiment-tracking service yet.

## Intended design boundaries

- Controllers should depend on typed state and reference interfaces, not on a
  particular simulator or hardware API.
- Controller-requested torque should remain distinct from the final applied
  torque boundary owned by future safety supervision.
- The safety supervisor should remain independently testable and independent of
  reinforcement-learning framework internals.
- Simulation and future physical adapters should implement a common plant-facing
  contract while keeping timing and transport details explicit.
- Experiment configuration and metric definitions should be version controlled.
- Training artifacts and large datasets should live outside the source tree and
  carry provenance that links them to code and configuration.

These boundaries are provisional and may change through documented Architecture
Decision Records.
