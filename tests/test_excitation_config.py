"""Tests for strict excitation robustness configuration loading."""

import json
import math
from pathlib import Path
from typing import cast

import pytest

from adaptive_assist.evaluation import (
    DisturbanceProfile,
    ExcitationConfigError,
    load_excitation_sweep_config,
)

REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
CONFIG_PATH = REPOSITORY_ROOT / "configs/robustness/excitation_disturbance_sweep.json"


def _payload() -> dict[str, object]:
    raw: object = json.loads(CONFIG_PATH.read_text(encoding="utf-8"))
    return cast(dict[str, object], raw)


def _write(path: Path, payload: dict[str, object]) -> None:
    path.write_text(json.dumps(payload), encoding="utf-8")


def test_excitation_configuration_loads() -> None:
    configuration = load_excitation_sweep_config(CONFIG_PATH)

    assert configuration.reference_frequencies_hz == (0.25, 0.5, 0.75, 1.0)
    assert configuration.inertia_scale_factors == (0.8, 1.0, 1.2)
    assert configuration.reference_amplitude_rad == pytest.approx(0.2)
    assert configuration.disturbance_profiles == tuple(DisturbanceProfile)
    assert configuration.disturbance_end_time_s == pytest.approx(2.25)


def test_malformed_excitation_configuration_is_rejected(tmp_path: Path) -> None:
    path = tmp_path / "malformed.json"
    path.write_text("{bad JSON", encoding="utf-8")

    with pytest.raises(ExcitationConfigError, match="Invalid JSON"):
        load_excitation_sweep_config(path)


@pytest.mark.parametrize(
    ("field", "value"),
    [
        ("reference_frequencies_hz", [0.0, 0.5]),
        ("reference_frequencies_hz", [0.5, 0.5]),
        ("inertia_scale_factors", [0.8, 1.2]),
        ("inertia_scale_factors", [-0.8, 1.0, 1.2]),
        ("reference_amplitude_rad", 0.0),
        ("disturbance_pulse_magnitude_n_m", -1.0),
        ("disturbance_start_time_s", -0.1),
        ("disturbance_duration_s", 0.0),
        ("recovery_tolerance_rad", 0.0),
        ("recovery_dwell_duration_s", 0.0),
        ("reference_offset_rad", math.inf),
    ],
)
def test_invalid_numeric_configuration_is_rejected(
    tmp_path: Path, field: str, value: object
) -> None:
    payload = _payload()
    payload[field] = value
    path = tmp_path / "invalid.json"
    _write(path, payload)

    with pytest.raises(ExcitationConfigError, match=field):
        load_excitation_sweep_config(path)


def test_missing_field_is_rejected(tmp_path: Path) -> None:
    payload = _payload()
    del payload["reference_frequencies_hz"]
    path = tmp_path / "missing.json"
    _write(path, payload)

    with pytest.raises(ExcitationConfigError, match="missing required field"):
        load_excitation_sweep_config(path)


def test_unknown_field_is_rejected(tmp_path: Path) -> None:
    payload = _payload()
    payload["random_seed"] = 42
    path = tmp_path / "unknown.json"
    _write(path, payload)

    with pytest.raises(ExcitationConfigError, match="unsupported field"):
        load_excitation_sweep_config(path)


def test_unknown_disturbance_profile_is_rejected(tmp_path: Path) -> None:
    payload = _payload()
    payload["disturbance_profiles"] = ["none", "random"]
    path = tmp_path / "unknown-profile.json"
    _write(path, payload)

    with pytest.raises(ExcitationConfigError, match="must be one of"):
        load_excitation_sweep_config(path)
