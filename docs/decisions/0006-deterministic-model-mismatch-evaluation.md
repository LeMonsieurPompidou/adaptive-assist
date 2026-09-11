# ADR 0006: Deterministic Model-Mismatch Evaluation

- **Status:** Accepted
- **Date:** 2026-09-11

## Context

The scalar plant, two fixed-gain classical controllers, deterministic
experiment framework, and simulation supervisor are implemented. Computed
torque assumes nominal dynamics, but nominal comparisons cannot show how model
error affects that compensation. A reproducible mismatch baseline is needed
before considering stochastic randomization or residual learning.

## Decision

Add a focused evaluation layer that keeps the computed-torque nominal model
fixed while constructing separately validated actual plants from multiplicative
parameter changes. Evaluate inertia, mass, centre-of-mass distance, damping,
and stiffness one at a time at 0.8, 1.0, and 1.2 times nominal, plus one named
moderate combined case.

Use the same scenario, feedback gains, and optional simulation supervisor for
both controllers. Impedance receives no plant model. Execute the nominal case
only once per controller, retain existing `ExperimentResult` values, reuse
existing metrics, and add a relative RMSE degradation summary. Store the strict
configuration in JSON and export summaries with the standard library only.

The main configured evaluation is supervised. Preserve a separately labelled
unsupervised mode through the runner's existing optional-supervisor boundary so
command clipping does not obscure all raw-controller interpretation.

## Rationale

- Deterministic sweeps make each changed assumption visible and debuggable
  before uncertainty distributions are defensible.
- Separate nominal and actual objects prevent perturbed plant knowledge from
  leaking into computed-torque compensation.
- One-at-a-time cases identify first-order sensitivity without requiring a
  general benchmark or combinatorial design.
- Fixed, equal feedback gains avoid per-case tuning and keep the initial
  controller comparison interpretable.
- Reusing the same supervisor preserves the implemented deployment-order
  boundary while explicit unsupervised runs expose raw command behavior.
- Existing experiment records and metrics are sufficient; composition avoids
  duplicating sample data or dynamics equations.
- Deferring reinforcement learning keeps this milestone focused on defining
  and measuring the deterministic robustness problem.

## Consequences

- Computed-torque runs carry both actual plant parameters and fixed nominal
  controller-model parameters in their summaries; impedance nominal-model data
  is explicitly absent.
- Each configured sweep runs 12 unique cases per controller: one nominal, ten
  one-at-a-time mismatches, and one combined mismatch.
- The console repeats the one executed nominal result as each family's 1.0 row;
  summary CSV stores only unique executions.
- Supervised and unsupervised outputs must be interpreted separately.
- Relative RMSE degradation is descriptive and deterministic, not an
  uncertainty estimate or evidence of real-world robustness.
- The parameter factors are illustrative and cannot be described as identified
  human or hardware uncertainty.

## Alternatives considered

- **Random or domain-randomized sampling:** deferred until distributions and
  reproducibility requirements are justified.
- **Full-factorial parameter combinations:** deferred because they increase
  run count and make first-order interpretation harder.
- **Retune each controller per case:** rejected because it changes the question
  from robustness of fixed baselines to case-specific optimization.
- **Give computed torque the perturbed plant parameters:** rejected because it
  removes the model-mismatch condition being evaluated.
- **Put robustness logic in the generic runner or plant:** rejected because
  parameter-study orchestration is an evaluation concern.
- **Add NumPy, pandas, Hydra, or experiment tracking:** rejected as unnecessary
  for this small deterministic scalar sweep.

## Reconsider when

- evidence supports specific uncertainty ranges or correlated distributions;
- disturbance suites and parameter mismatch need a shared evaluation plan;
- runtime or case count justifies batch or tabular dependencies;
- multiple scenarios require richer provenance or artifact management;
- domain randomization requirements are defined; or
- a residual policy has a fixed observation, authority, objective, and
  evaluation protocol.
