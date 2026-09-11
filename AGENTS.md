# Repository guidance for Codex

## Purpose and phase

`adaptive-assist` is a research and portfolio project for safe learning-based
control and sim-to-real evaluation of a simplified assistive joint. The project
has completed its foundation, deterministic plant-model, experiment-interface,
impedance-controller, computed-torque-controller, and deterministic simulation
safety-supervisor milestones, plus deterministic model-mismatch evaluation. A
scalar 1-DOF mathematical model, deterministic experiment framework, two
unsaturated classical baselines, an independent command-constraint layer, and
fixed one-at-a-time robustness sweeps exist. MPC, reinforcement learning,
domain randomization, external simulation, ROS 2, hardware functionality, and
real-world safety validation do not.

## Key directories

- `src/adaptive_assist/`: deterministic dynamics, controller, safety,
  experiment, and evaluation interfaces.
- `tests/`: automated tests.
- `docs/`: scope, architecture, and decision records.
- `configs/`: version-controlled experiment scenarios and controller gains.
- `scripts/`: development and environment utilities.
- `assets/`: documentation media, not experiment data.

## Working rules

- Keep changes small, focused, and reviewable.
- Use type annotations for Python code and add tests for meaningful behavior.
- Update documentation whenever the architecture or scope changes.
- Clearly distinguish implemented behavior from planned behavior.
- Never invent benchmark or performance results.
- Never present this project as a medical device or as suitable for human use.
- Keep controller-requested torque distinct from safety-supervised applied
  torque; do not add safety behavior inside controllers or the plant.
- Treat configured safety limits as mathematical demonstration values, never as
  human, device, medical, or clinical thresholds.
- Keep actual perturbed plant parameters separate from the fixed nominal model
  owned by computed-torque control; never retune a baseline per mismatch case.
- Treat configured robustness ranges as illustrative engineering values, not
  identified uncertainty distributions or evidence that learning is needed.
- Do not add major dependencies without documenting the decision in an ADR.
- Run all validation commands before finishing:
  - `python -m ruff format --check .`
  - `python -m ruff check .`
  - `python -m mypy`
  - `python -m pytest`
