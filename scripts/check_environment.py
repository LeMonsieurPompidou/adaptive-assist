"""Report whether the local environment can use the project foundation."""

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
    "pyproject.toml",
    "docs/architecture.md",
    "docs/decisions/0001-simulator-independent-one-dof-model.md",
    "docs/decisions/README.md",
    "docs/one_dof_model.md",
    "docs/project_scope.md",
    "scripts/run_free_joint_demo.py",
    "src/adaptive_assist/__init__.py",
    "src/adaptive_assist/__main__.py",
    "src/adaptive_assist/dynamics/__init__.py",
    "src/adaptive_assist/dynamics/joint.py",
    "src/adaptive_assist/main.py",
    "src/adaptive_assist/py.typed",
    "tests/test_joint_dynamics.py",
    "tests/test_package.py",
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
    print("Expected foundation files:")
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
