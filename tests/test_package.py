"""Tests for package metadata and the status entry point."""

import os
import subprocess
import sys
from pathlib import Path

import adaptive_assist
from adaptive_assist.main import FOUNDATION_MESSAGE


def test_package_imports_and_exposes_version() -> None:
    """The package should import and expose a non-empty version string."""
    assert isinstance(adaptive_assist.__version__, str)
    assert adaptive_assist.__version__


def test_module_entry_point() -> None:
    """The module entry point should run and describe current project status."""
    repository_root = Path(__file__).resolve().parents[1]
    environment = os.environ.copy()
    source_path = str(repository_root / "src")
    existing_pythonpath = environment.get("PYTHONPATH")
    environment["PYTHONPATH"] = (
        source_path
        if existing_pythonpath is None
        else os.pathsep.join((source_path, existing_pythonpath))
    )

    result = subprocess.run(
        [sys.executable, "-m", "adaptive_assist"],
        cwd=repository_root,
        env=environment,
        check=False,
        capture_output=True,
        text=True,
    )

    assert result.returncode == 0, result.stderr
    assert result.stdout.strip() == FOUNDATION_MESSAGE
    assert "deterministic 1-DOF joint model" in result.stdout
    assert "impedance controller" in result.stdout
    assert "computed-torque controller" in result.stdout
    assert "MPC and safety supervision are not yet implemented" in result.stdout
