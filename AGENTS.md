# Repository guidance for Codex

## Purpose and phase

`adaptive-assist` is a research and portfolio project for safe learning-based
control and sim-to-real evaluation of a simplified assistive joint. The project
has completed its foundation, deterministic plant-model, and experiment-
interface milestones. A scalar 1-DOF mathematical model and deterministic
open-loop experiment framework exist, but controllers, safety supervision,
reinforcement learning, external simulation, ROS 2, and hardware functionality
do not.

## Key directories

- `src/adaptive_assist/`: deterministic dynamics and experiment interfaces.
- `tests/`: automated tests.
- `docs/`: scope, architecture, and decision records.
- `configs/`: version-controlled experiment scenarios.
- `scripts/`: development and environment utilities.
- `assets/`: documentation media, not experiment data.

## Working rules

- Keep changes small, focused, and reviewable.
- Use type annotations for Python code and add tests for meaningful behavior.
- Update documentation whenever the architecture or scope changes.
- Clearly distinguish implemented behavior from planned behavior.
- Never invent benchmark or performance results.
- Never present this project as a medical device or as suitable for human use.
- Do not add major dependencies without documenting the decision in an ADR.
- Run all validation commands before finishing:
  - `python -m ruff format --check .`
  - `python -m ruff check .`
  - `python -m mypy`
  - `python -m pytest`
