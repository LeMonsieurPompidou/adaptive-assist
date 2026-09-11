"""Report whether the local environment can use the current project."""

import platform
import sys
from pathlib import Path

MINIMUM_PYTHON = (3, 11)
EXPECTED_FILES = (
    ".github/workflows/ci.yml",
    ".editorconfig",
    ".gitattributes",
    ".gitignore",
    "AGENTS.md",
    "LICENSE",
    "README.md",
    "assets/README.md",
    "configs/README.md",
    "configs/controllers/computed_torque_baseline.json",
    "configs/controllers/impedance_baseline.json",
    "configs/scenarios/nominal_open_loop.json",
    "configs/scenarios/nominal_tracking.json",
    "pyproject.toml",
    "docs/architecture.md",
    "docs/decisions/0001-simulator-independent-one-dof-model.md",
    "docs/decisions/0002-deterministic-experiment-framework.md",
    "docs/decisions/0003-impedance-controller-baseline.md",
    "docs/decisions/0004-computed-torque-model-based-baseline.md",
    "docs/decisions/README.md",
    "docs/computed_torque_controller.md",
    "docs/developer_guide.md",
    "docs/experiment_framework.md",
    "docs/impedance_controller.md",
    "docs/one_dof_model.md",
    "docs/project_scope.md",
    "scripts/run_free_joint_demo.py",
    "scripts/run_computed_torque_experiment.py",
    "scripts/run_impedance_experiment.py",
    "scripts/run_open_loop_experiment.py",
    "scripts/compare_baseline_controllers.py",
    "scripts/compare_open_loop_impedance.py",
    "src/adaptive_assist/__init__.py",
    "src/adaptive_assist/__main__.py",
    "src/adaptive_assist/controllers/__init__.py",
    "src/adaptive_assist/controllers/base.py",
    "src/adaptive_assist/controllers/computed_torque.py",
    "src/adaptive_assist/controllers/computed_torque_config.py",
    "src/adaptive_assist/controllers/impedance.py",
    "src/adaptive_assist/controllers/impedance_config.py",
    "src/adaptive_assist/dynamics/__init__.py",
    "src/adaptive_assist/dynamics/joint.py",
    "src/adaptive_assist/experiments/__init__.py",
    "src/adaptive_assist/experiments/logging.py",
    "src/adaptive_assist/experiments/metrics.py",
    "src/adaptive_assist/experiments/records.py",
    "src/adaptive_assist/experiments/reference.py",
    "src/adaptive_assist/experiments/runner.py",
    "src/adaptive_assist/experiments/scenario.py",
    "src/adaptive_assist/main.py",
    "src/adaptive_assist/py.typed",
    "tests/test_joint_dynamics.py",
    "tests/test_closed_loop_runner.py",
    "tests/test_computed_torque_config.py",
    "tests/test_computed_torque_controller.py",
    "tests/test_computed_torque_integration.py",
    "tests/test_controller_comparison.py",
    "tests/test_experiment_logging.py",
    "tests/test_experiment_metrics.py",
    "tests/test_experiment_reference.py",
    "tests/test_experiment_runner.py",
    "tests/test_impedance_config.py",
    "tests/test_impedance_controller.py",
    "tests/test_package.py",
    "tests/test_scenario_config.py",
)


def main() -> int:
    """Print environment information and return whether essential checks pass."""
    repository_root = Path.cwd()
    python_supported = sys.version_info >= MINIMUM_PYTHON
    root_markers = (repository_root / "pyproject.toml", repository_root / ".git")
    at_repository_root = all(marker.exists() for marker in root_markers)
    missing_files = [
        relative_path
        for relative_path in EXPECTED_FILES
        if not (repository_root / relative_path).is_file()
    ]

    print(f"Operating system: {platform.platform()}")
    print(f"Python version: {platform.python_version()}")
    print(f"Python executable: {sys.executable}")
    print(f"Current working directory: {repository_root}")
    print(f"Opened from repository root: {'yes' if at_repository_root else 'no'}")
    print(f"Python >= 3.11: {'yes' if python_supported else 'no'}")
    print("Expected project files:")
    for relative_path in EXPECTED_FILES:
        status = "present" if (repository_root / relative_path).is_file() else "missing"
        print(f"  [{status}] {relative_path}")

    if missing_files:
        print(
            f"Environment check failed: {len(missing_files)} essential file(s) missing."
        )
    if not python_supported:
        print("Environment check failed: Python 3.11 or newer is required.")

    return 1 if missing_files or not python_supported else 0


if __name__ == "__main__":
    raise SystemExit(main())
