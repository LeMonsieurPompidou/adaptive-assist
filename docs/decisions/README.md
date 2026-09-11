# Architecture Decision Records

Significant technical decisions will be documented here as Architecture
Decision Records (ADRs). Each record should state its context, considered
options, decision, consequences, and status so that the project's evolution is
reviewable.

Likely ADR topics include:

- simulator selection;
- Python version strategy;
- reinforcement-learning framework;
- ROS 2 integration;
- experiment-tracking tool; and
- sim-to-real strategy.

Accepted records:

- [ADR 0001: Simulator-independent one-DOF plant model](0001-simulator-independent-one-dof-model.md)
- [ADR 0002: Deterministic experiment framework](0002-deterministic-experiment-framework.md)
- [ADR 0003: Impedance controller baseline](0003-impedance-controller-baseline.md)
- [ADR 0004: Computed-torque model-based baseline](0004-computed-torque-model-based-baseline.md)
- [ADR 0005: Independent simulation safety supervisor](0005-independent-safety-supervisor.md)
- [ADR 0006: Deterministic model-mismatch evaluation](0006-deterministic-model-mismatch-evaluation.md)

Adding a major dependency or committing to another listed technology should be
accompanied by an ADR.
