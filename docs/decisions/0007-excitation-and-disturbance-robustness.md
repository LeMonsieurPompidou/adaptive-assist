# ADR 0007: Excitation and Disturbance Robustness

- **Status:** Accepted
- **Date:** 2026-10-12

## Context

The first deterministic model-mismatch sweep found that actual inertia was the
strongest tested parameter sensitivity, while the configured damping and
stiffness changes had smaller effects. A single `0.5 Hz` trajectory could not
show whether inertia sensitivity grows with demanded acceleration or how
external disturbances interact with the fixed safety constraint.

For a sinusoidal reference, desired acceleration magnitude scales with
frequency squared. Computed-torque control uses nominal inertia multiplied by
desired acceleration, while impedance control relies only on fixed-gain error
feedback. A deterministic frequency and pulse study is therefore the next
focused comparison before considering learning or stochastic evaluation.

## Decision

The project will:

- evaluate fixed-amplitude sinusoidal references at `0.25`, `0.50`, `0.75`,
  and `1.00 Hz`;
- vary only actual inertia by `0.8x`, `1.0x`, and `1.2x` while retaining the
  computed-torque controller's fixed nominal model;
- apply no disturbance, a positive `1 N m` pulse, or a negative `1 N m` pulse
  over the same half-open time interval;
- keep controller gains, initial conditions, timing, human torque, and safety
  limits fixed across corresponding cases;
- use the existing generic runner through a small deterministic `TorqueSignal`
  input rather than adding evaluation behavior to controllers or the plant;
- retain the existing safety supervisor for the primary evaluation and expose
  an explicit unsupervised diagnostic mode;
- add peak absolute tracking error and a fixed-step, dwell-based recovery metric;
  and
- retain optional standard-library CSV output.

## Rationale

Deterministic pulses make sign, timing, and expected plant input directly
auditable before any stochastic disturbance process is considered. Fixed gains
preserve comparison fairness and prevent per-case tuning from hiding the
frequency/inertia interaction. Reusing the unchanged supervisor tests the
currently deployed mathematical command path and allows a separate
unsupervised run to expose clipping effects.

Reinforcement learning remains deferred because the experiment identifies a
control gap but does not establish that learning is the necessary or best
remedy. The observation, residual authority, baseline-relative objective, and
evaluation protocol still require separate decisions.

## Consequences

- Each mode executes 72 deterministic controller experiments.
- Disturbance torque can vary with time without changing controller interfaces.
- Previous constant-disturbance scenarios retain their behavior.
- Recovery is explicitly sample-defined and may fail because baseline periodic
  tracking error remains outside the absolute tolerance band.
- The `2 N m` supervisor limit can dominate high-frequency results, so
  supervised and unsupervised conclusions must remain separate.
- The ranges are illustrative engineering inputs, not identified uncertainty or
  human-interaction distributions.

## Alternatives considered

- **Random disturbance sampling:** rejected because it would add seeds,
  distributions, and statistical interpretation before deterministic behavior
  is understood.
- **Retuning gains per frequency:** rejected because it would confound the
  excitation effect with controller tuning.
- **Controller-specific disturbance compensation:** rejected because the study
  concerns the existing baselines and external disturbances are not controller
  inputs.
- **A new benchmark runner:** rejected because the existing experiment runner
  already owns reference evaluation, controller execution, supervision,
  records, and plant stepping.
- **Immediate residual learning:** deferred because deterministic evidence and
  the residual-learning specification remain incomplete.

## Reconsideration criteria

Revisit this decision if evidence-based uncertainty ranges become available,
if disturbance phase or amplitude studies require a richer declarative signal,
if continuous-time recovery must be estimated between samples, or if an
external simulator introduces a native disturbance/logging interface. Any
stochastic extension must define distributions, seeds, replication, and
statistical reporting explicitly.
