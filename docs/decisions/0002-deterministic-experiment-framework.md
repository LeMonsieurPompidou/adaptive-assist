# ADR 0002: Deterministic experiment framework

- **Status:** Accepted
- **Date:** 2026-09-10

## Context

The scalar 1-DOF plant needs a reproducible way to define inputs, execute
fixed-step scenarios, preserve observations, and calculate initial metrics
before controllers are introduced. The framework must remain inspectable and
simulator-independent while the project has no need for distributed runs,
large tabular datasets, or third-party experiment tracking.

## Decision

Implement the first experiment layer with:

- immutable typed scenario, reference, sample, result, and metadata records;
- deterministic fixed-step open-loop execution;
- strict version-controlled JSON scenario files;
- standard-library CSV sample logging;
- controller-independent metric functions; and
- no pandas, configuration framework, or experiment-tracking dependency.

Scenario metadata contains deterministic identifiers and timing values. It does
not contain timestamps, UUIDs, or randomness. Configured torques are predefined
open-loop inputs, not controller outputs.

## Reasons

- **Explicit typed records** make units and data ownership reviewable and give
  future controllers a stable set of experiment-level concepts.
- **Fixed-step execution** matches the existing plant integrator and gives
  deterministic sample times and counts.
- **Version-controlled JSON** is human-readable, uses the standard library, and
  is sufficient for one strict schema and a small scenario set.
- **Standard-library CSV** creates portable outputs without a dataframe
  dependency.
- **Controller-independent metrics** allow the same definitions to compare
  future control approaches.
- **Deterministic execution** makes regressions and repeated runs easier to
  inspect.

## Consequences

Positive consequences:

- nominal experiments can be reproduced directly from repository content;
- records, logging, and metrics remain separate from plant dynamics;
- malformed and unsupported configurations fail explicitly; and
- identical scenarios produce structurally equal results.

Trade-offs:

- JSON schema parsing is intentionally explicit rather than generic;
- all samples are held in memory before CSV export;
- constant torques are the only implemented plant-input source;
- CSV does not manage artifacts or searchable run metadata; and
- adding feedback control will require a new torque-provider boundary or runner
  extension.

## Alternatives considered

- **YAML or TOML configuration:** not selected because JSON satisfies the
  current nested schema without another parser dependency.
- **Hydra or another configuration framework:** deferred until composition,
  sweeps, and override requirements justify it.
- **pandas logging:** deferred because the current scalar rows can be written by
  the standard `csv` module.
- **MLflow or Weights & Biases:** deferred until experiment volume, artifact
  tracking, and collaboration requirements are defined.
- **Simulator-native or ROS 2 logging:** deferred because neither an external
  simulator nor ROS 2 integration exists.
- **Wall-clock timestamps or UUIDs:** not selected because they make otherwise
  identical in-memory results differ without improving this milestone.

## Future reconsideration criteria

Revisit this decision when the project needs configuration composition, large
parameter sweeps, concurrent or distributed experiments, streaming datasets,
artifact provenance across machines, interactive dashboards, external
simulators, ROS 2 transport, or hardware-timed execution. The current choices
do not prevent later adoption of Hydra, pandas, MLflow, Weights & Biases, ROS 2,
or simulator-native logging through documented interfaces and migration ADRs.

