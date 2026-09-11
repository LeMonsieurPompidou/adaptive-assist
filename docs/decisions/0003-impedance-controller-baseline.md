# ADR 0003: Impedance Controller Baseline

- **Status:** Accepted
- **Date:** 2026-09-10

## Context

The project needs a first closed-loop baseline that is deterministic,
interpretable, simulator-independent, and compatible with the existing typed
plant and experiment records. It must establish the controller boundary before
model-based control, safety supervision, or learning is introduced.

## Decision

Implement impedance control first as explicit proportional position-error and
derivative velocity-error terms. The controller consumes `JointState` and
`JointReference` through a small typed `JointController` protocol and returns a
`ControllerOutput` containing requested assistive torque.

Store immutable, finite, non-negative gains in
`ImpedanceControllerParameters`. Keep their strict JSON configuration separate
from physical scenario configuration and version control the baseline values.

Do not saturate, filter, or otherwise alter the controller request inside the
controller. Until a safety supervisor is implemented, the closed-loop runner
passes requested assistive torque directly into the applied `JointTorques`
bundle. This pass-through is explicit and temporary.

## Rationale

- Proportional and derivative terms are easy to inspect, test, and relate to
  tracking behavior.
- The controller has no hidden state and remains exactly deterministic.
- A requested-torque output preserves a future boundary between control and
  safety supervision.
- Keeping safety constraints outside the controller prevents controller tuning
  from silently changing the final command policy.
- Version-controlled gains make demonstrations and future comparisons
  reproducible without hard-coded script parameters.
- Standard-library JSON preserves the repository's dependency-free runtime.

## Consequences

- The project now supports closed-loop impedance tracking and an equivalent-
  conditions comparison with open-loop assistance.
- Requested and applied assistive torque are numerically equal until a safety
  supervisor is introduced.
- Unbounded finite requests can be produced; this is an explicit limitation and
  must not be interpreted as safe for physical or human interaction.
- The same experiment records, CSV writer, and metrics support both open- and
  closed-loop results.
- A future model-based controller can implement the same small controller
  protocol without changing the plant.

## Alternatives considered

- **Embed feedback in the plant:** rejected because dynamics must remain
  independent of control policy.
- **Add saturation to the impedance controller:** rejected because final-command
  constraints belong to the future safety supervisor.
- **Use a generic controller/configuration framework:** deferred because one
  baseline does not justify the additional abstraction.
- **Implement model-based control first:** deferred because impedance control is
  the smaller interpretable baseline and establishes the interface needed by
  later controllers.

## Reconsider when

- a safety supervisor defines requested-versus-applied command records;
- controller diagnostics require a small shared representation;
- multiple controller configurations justify shared parsing infrastructure;
- state estimation changes the controller input contract; or
- actuator or timing models make a stateless per-sample interface insufficient.
