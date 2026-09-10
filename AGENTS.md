# Repository guidance for Codex

## Purpose and phase

`adaptive-assist` is a research and portfolio project for safe learning-based
control and sim-to-real evaluation of a simplified assistive joint. The project
has completed its foundation, deterministic plant-model, experiment-interface,
and impedance-controller milestones. A scalar 1-DOF mathematical model,
deterministic experiment framework, and unsaturated impedance baseline exist.
Model-based control, safety supervision, reinforcement learning, external
simulation, ROS 2, and hardware functionality do not.

## Key directories

- `src/adaptive_assist/`: deterministic dynamics, controller, and experiment
  interfaces.
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
- Keep controller-requested torque distinct from future safety-supervised and
  applied torque; do not add safety behavior inside controllers or the plant.
- Do not add major dependencies without documenting the decision in an ADR.
- Run all validation commands before finishing:
  - `python -m ruff format --check .`
  - `python -m ruff check .`
  - `python -m mypy`
  - `python -m pytest`
