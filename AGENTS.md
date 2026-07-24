# Repository guidance for Codex

## Purpose and phase

`adaptive-assist` is a research and portfolio project for safe learning-based
control and sim-to-real evaluation of a simplified assistive joint. The project
is currently in its foundation phase: packaging, documentation, and quality
tooling exist, but simulation and control functionality do not.

## Key directories

- `src/adaptive_assist/`: installable Python package.
- `tests/`: automated tests.
- `docs/`: scope, architecture, and decision records.
- `configs/`: future version-controlled experiment configuration.
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

