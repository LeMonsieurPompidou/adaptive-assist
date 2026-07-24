# Adaptive Assist

Safe learning-based control and sim-to-real evaluation for a simplified
assistive robotics joint.

> **Project status: Work in progress** — the repository currently provides the
> project foundation only; no physical simulation, controller, learned policy,
> ROS 2 interface, or hardware interface has been implemented.

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

### Planned controller comparison

| Controller | Planned role | Status |
| --- | --- | --- |
| Classical impedance controller | Interpretable baseline with tunable compliance | Planned |
| Model-based controller | Dynamics-aware reference for performance and robustness | Planned |
| Safe hybrid residual RL controller | Learned bounded correction on top of a baseline controller | Planned |

No controller in this table is implemented yet.

## Safety positioning

Safety is an architectural constraint, not a claim of validation. Planned work
includes explicit state and actuator limits, a supervisor around learned
actions, bounded residual authority, violation logging, deterministic fallback
behavior, and evaluation under disturbances and model uncertainty. These
mechanisms will require evidence in simulation and controlled bench testing;
their presence alone would not establish safety for people.

## Planned technical stack

- **Language and tooling:** Python 3.11+, pytest, Ruff, and mypy (implemented).
- **Dynamics simulation:** undecided; MuJoCo and other options will be evaluated
  in an Architecture Decision Record (tentative).
- **Reinforcement learning:** framework and algorithm undecided (tentative).
- **Experiment tracking:** tool undecided; reproducible file-based logging may
  be used first (tentative).
- **Robotics integration:** ROS 2 may be introduced in a later phase
  (tentative).
- **Hardware:** a non-human-worn one-degree-of-freedom bench setup may be
  considered only after simulation milestones (tentative).

Major technology choices are deliberately deferred until their requirements
and trade-offs can be documented.

## Current capabilities

At this foundation stage, the repository provides:

- an installable, typed Python `src`-layout package;
- a module entry point that reports the current implementation status;
- automated formatting, linting, type-checking, and tests;
- a cross-platform environment checker;
- continuous integration across supported Python versions; and
- project scope, planned architecture, and decision-record documentation.

## Roadmap

1. **Foundation:** package structure, engineering documentation, quality gates,
   and CI.
2. **Simulation:** define equations, parameters, reference trajectories,
   disturbances, and a deterministic one-degree-of-freedom environment.
3. **Classical baselines:** implement and test impedance and model-based
   controllers.
4. **Safety layer:** implement constraints, action filtering, fallback behavior,
   and violation reporting.
5. **Residual learning:** train bounded residual policies and compare them with
   the baselines.
6. **Robust evaluation:** run parameter sweeps, disturbance tests, ablations,
   and reproducible benchmark reports.
7. **Sim-to-real preparation:** document the transfer strategy and, if
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
```

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
├── configs/                    # Future experiment configuration
├── docs/                       # Scope, architecture, and decision records
├── scripts/check_environment.py
├── src/adaptive_assist/        # Installable package
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
