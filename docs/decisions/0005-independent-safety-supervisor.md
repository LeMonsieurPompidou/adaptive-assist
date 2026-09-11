# ADR 0005: Independent Simulation Safety Supervisor

- **Status:** Accepted
- **Date:** 2026-09-11

## Context

The project now has two controllers that produce requested assistive torque,
but their requests have previously passed directly to the scalar plant. Future
residual-learning work also needs a stable downstream boundary that can record
controller intent separately from the command permitted by deterministic
simulation constraints.

The first supervisor must be easy to test and interpret without introducing
predictive control, actuator modeling, hardware behavior, or claims about
human safety.

## Decision

Implement one immutable `SafetySupervisor` outside controllers and the plant.
It receives current `JointState` plus requested assistive torque and returns an
immutable `SafetyResult` with requested torque, applied torque, and ordered
intervention reasons.

Use deterministic precedence:

1. replace a non-finite request with configured fallback;
2. replace the request with fallback when current position and/or velocity is
   outside configured bounds; and
3. otherwise clip requested torque symmetrically when required.

Keep the fallback within the configured torque magnitude. Retain optional
direct pass-through in the generic runner and mark supervision status in
experiment metadata. Store requested torque and intervention reasons in each
sample while retaining applied assistive torque in `JointTorques`.

Use strict version-controlled JSON and standard-library parsing. Treat every
configured value as an illustrative simulation limit, not a human, device, or
clinical threshold.

## Rationale

- Independence applies the same rules to impedance, computed-torque, and future
  upstream command sources.
- Separating requested and applied values makes interventions auditable and
  supports future residual-policy evaluation.
- Clipping and fixed fallback are explicit and deterministic enough for the
  current mathematical model.
- State checks prevent an already out-of-range state from receiving a normal
  controller request without pretending to calculate a recovery trajectory.
- Standard-library JSON preserves the dependency-free runtime.
- Explicit metadata avoids confusing an inactive supervisor with an active
  supervisor that happened not to intervene.

## Consequences

- `run_closed_loop_experiment()` has an optional supervisor argument but remains
  generic across controllers.
- Experiment samples and CSV rows now distinguish requested from applied
  assistive torque and retain intervention reasons.
- Metrics can count and quantify command modifications independently of
  controller implementation.
- The final sample resolves and records a command without integrating another
  interval; sample-level metrics include that evaluation.
- Existing unsupervised callers retain direct-pass-through behavior.
- Simple fallback does not ensure recovery or future-state feasibility.
- The existence of this layer provides no evidence of real-world safety.

## Alternatives considered

- **Embed limits in each controller:** rejected because rules could diverge and
  future learned commands could bypass them.
- **Put limits in the plant:** rejected because command policy is separate from
  dynamics.
- **Remove unsupervised execution:** deferred because reproducible controller
  baselines and explicit supervision-effect comparisons remain useful.
- **Add a generic safety framework:** rejected as unnecessary for the current
  small set of deterministic constraints.
- **Implement control barrier functions or predictive constraints:** deferred
  until state constraints, actuator behavior, and evaluation requirements are
  defined.

## Reconsider when

- actuator dynamics or torque-rate constraints are modeled;
- predictive feasibility or recovery behavior is required;
- learned residual commands and their authority are defined;
- requested and applied records need interval-aware energy accounting;
- external simulator or hardware adapters introduce timing and fault semantics;
  or
- evidence supports a separately governed real-world validation program.
