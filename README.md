# Adaptive Assist

Safe learning-based control and sim-to-real evaluation for a simplified
assistive robotics joint.

> **Project status: Work in progress** — a deterministic, simulator-independent
> one-degree-of-freedom mathematical joint model, experiment framework,
> impedance controller, computed-torque controller, and a deterministic
> simulation safety supervisor are implemented. MPC, learned policies, external
> simulation, ROS 2, and hardware interfaces remain planned.

## Motivation

Assistive robots must combine useful physical assistance with predictable,
constraint-aware behavior. This project creates a transparent engineering
testbed for studying how classical control, model-based control, and learned
residual actions compare under the same assumptions and evaluation protocol.
Beginning with a one-degree-of-freedom abstraction keeps the dynamics and
safety boundaries understandable before any move toward more complex systems.

## Long-term objectives

The project is intended to:

- model a one-degree-of-freedom lower-limb assistive joint in simulation;
- establish reproducible trajectories, disturbances, and parameter variations;
- compare tracking, effort, energy, smoothness, safety, and robustness;
- investigate safety-supervised residual reinforcement learning;
- define a staged sim-to-real evaluation process; and
- document engineering decisions and limitations clearly.

### Controller comparison roadmap

| Controller | Role | Status |
| --- | --- | --- |
| Classical impedance controller | Interpretable baseline with tunable compliance | Implemented |
| Computed-torque controller | Model-based baseline with nominal dynamics compensation | Implemented |
| Model predictive control (MPC) | Constrained optimization-based baseline | Planned |
| Safe hybrid residual RL controller | Learned bounded correction on top of a baseline controller | Planned |

Both implemented controllers produce unsaturated requested torque. The optional
simulation supervisor independently resolves that request into applied torque;
direct pass-through remains available for explicit unsupervised comparisons.

## Safety positioning

Safety is an architectural constraint, not a claim of validation. The current
mathematical supervisor implements finite-command fallback, current-state angle
and velocity checks, symmetric torque clipping, and intervention records using
illustrative limits. Predictive constraints, torque-rate limits, bounded
residual authority, richer fallback behavior, and evaluation under uncertainty
remain planned. This simulation layer does not establish safety for people,
medical use, clinical use, or physical deployment.

## Planned technical stack

- **Language and tooling:** Python 3.11+, pytest, Ruff, and mypy (implemented).
- **Joint dynamics:** a scalar standard-library 1-DOF model with semi-implicit
  Euler integration is implemented.
- **Experiment infrastructure:** typed fixed-step scenarios, JSON configuration,
  immutable records, CSV export, and initial metrics are implemented.
- **Simulation command constraints:** deterministic requested/applied torque
  separation, clipping, fallback, intervention logging, and initial supervisor
  metrics are implemented with illustrative limits.
- **External dynamics simulation:** undecided; MuJoCo and other options will be
  evaluated in a future Architecture Decision Record (tentative).
- **Reinforcement learning:** framework and algorithm undecided (tentative).
- **Experiment tracking:** explicit standard-library CSV export is implemented;
  any broader tracking tool remains undecided (tentative).
- **Robotics integration:** ROS 2 may be introduced in a later phase
  (tentative).
- **Hardware:** a non-human-worn one-degree-of-freedom bench setup may be
  considered only after simulation milestones (tentative).

Major technology choices are deliberately deferred until their requirements
and trade-offs can be documented.

## Current capabilities

At the current simulation-safety-supervisor milestone, the repository provides:

- an installable, typed Python `src`-layout package;
- a validated deterministic 1-DOF rotational joint model using SI units;
- gravity, passive-torque, applied-torque, acceleration, and semi-implicit Euler
  step APIs;
- a lightweight constant-torque mathematical demonstration;
- version-controlled constant-torque open-loop scenarios with constant or
  sinusoidal references;
- deterministic experiment records and metadata, explicit CSV export,
  trajectory-tracking RMSE, and peak-assistive-torque metrics;
- a typed proportional-derivative impedance controller that produces requested
  assistive torque without saturation or safety filtering;
- a typed computed-torque controller that reuses the public plant model for
  nominal inertial feedforward, gravity compensation, and passive compensation;
- deterministic closed-loop execution and equivalent-conditions comparisons
  of zero-assistance open loop, impedance control, and computed-torque control;
- an optional controller-independent simulation safety supervisor with strict
  limits, deterministic precedence, requested/applied command records, and
  intervention metrics;
- automated formatting, linting, type-checking, and tests;
- a cross-platform environment checker;
- continuous integration across supported Python versions; and
- model, scope, planned architecture, and decision-record documentation.

## Roadmap

1. **Foundation — complete:** package structure, engineering documentation,
   quality gates, and CI.
2. **Deterministic joint model — complete:** validated scalar equations,
   fixed-step integration, unit tests, and a mathematical demonstration.
3. **Experiment interfaces — complete:** reference signals, versioned scenarios,
   deterministic execution, records, CSV logging, and initial metrics.
4. **Classical baselines — complete for the current scope:** impedance and
   computed-torque controllers are implemented; MPC remains planned.
5. **Simulation safety layer — complete for the current scope:** finite-command
   fallback, current-state checks, torque clipping, and intervention reporting.
6. **Residual learning:** train bounded residual policies and compare them with
   the baselines.
7. **Robust evaluation:** run parameter sweeps, disturbance tests, ablations,
   and reproducible benchmark reports.
8. **Sim-to-real preparation:** document the transfer strategy and, if
   justified, evaluate on an isolated bench apparatus without a human wearer.

## Installation

Python 3.11 or newer is required. From the repository root, create and activate
a virtual environment, then install the package and development tools:

```bash
python -m venv .venv
```

On Linux or macOS:

```bash
source .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install -e ".[dev]"
```

On Windows PowerShell:

```powershell
.venv\Scripts\Activate.ps1
python -m pip install --upgrade pip
python -m pip install -e ".[dev]"
```

Confirm the current package state:

```bash
python -m adaptive_assist
python scripts/check_environment.py
python scripts/run_free_joint_demo.py
python scripts/run_open_loop_experiment.py
python scripts/run_impedance_experiment.py
python scripts/run_computed_torque_experiment.py
python scripts/compare_baseline_controllers.py
python scripts/run_safety_supervisor_demo.py
python scripts/compare_open_loop_impedance.py
```

## Documentation

For a code-oriented tour of the repository, see the
[developer reading guide](docs/developer_guide.md).
The [experiment framework guide](docs/experiment_framework.md) documents the
scenario schema, execution records, CSV format, metrics, and boundaries.
The [impedance controller guide](docs/impedance_controller.md) documents the
implemented feedback law, gains, closed-loop flow, and safety boundary.
The [computed-torque controller guide](docs/computed_torque_controller.md)
documents the implemented nominal-model compensation, assumptions, and model
mismatch boundary.
The [simulation safety-supervisor guide](docs/safety_supervisor.md) documents
the implemented limits, precedence, command records, and validation boundary.

## Development and validation

Run the complete local quality suite from the repository root:

```bash
python -m ruff format --check .
python -m ruff check .
python -m mypy
python -m pytest
```

To apply Ruff formatting before re-running the checks:

```bash
python -m ruff format .
```

## Repository structure

```text
adaptive-assist/
├── .github/workflows/ci.yml    # Continuous integration
├── assets/                     # Documentation media
├── configs/                    # Scenarios, controller gains, and safety limits
├── docs/                       # Scope, architecture, and decision records
├── scripts/                     # Environment check and runnable demonstrations
├── src/adaptive_assist/        # Dynamics, controllers, safety, and experiments
├── tests/                      # Automated tests
├── AGENTS.md                   # Repository guidance for coding agents
├── pyproject.toml              # Packaging and tool configuration
└── README.md
```

## Disclaimer

This repository is a research and engineering portfolio prototype. It is **not
a medical device**, is not medically or clinically validated, and is not
intended for human use, direct human testing, diagnosis, treatment, or control
of a device worn by a person.

## Author

Sam Rahnemayan

## License

This project is available under the [MIT License](LICENSE).
