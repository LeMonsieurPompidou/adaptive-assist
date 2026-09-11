"""Tests for strict simulation safety-limit configuration loading."""

import json
from pathlib import Path
from typing import cast

import pytest

from adaptive_assist.safety import (
    SAFETY_CONFIG_SCHEMA_VERSION,
    SafetyConfigError,
    load_safety_limits,
)

REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
CONFIG_PATH = REPOSITORY_ROOT / "configs/safety/nominal_limits.json"


def _payload() -> dict[str, object]:
    raw_payload: object = json.loads(CONFIG_PATH.read_text(encoding="utf-8"))
    return cast(dict[str, object], raw_payload)


def _write_payload(path: Path, payload: dict[str, object]) -> None:
    path.write_text(json.dumps(payload), encoding="utf-8")


def test_nominal_safety_configuration_loads() -> None:
    limits = load_safety_limits(CONFIG_PATH)

    assert SAFETY_CONFIG_SCHEMA_VERSION == 1
    assert limits.max_abs_assistive_torque_n_m == pytest.approx(2.0)
    assert limits.min_joint_angle_rad == pytest.approx(-0.5)
    assert limits.max_joint_angle_rad == pytest.approx(0.5)
    assert limits.max_abs_joint_velocity_rad_s == pytest.approx(1.5)
    assert limits.fallback_assistive_torque_n_m == pytest.approx(0.0)


def test_malformed_safety_configuration_is_rejected(tmp_path: Path) -> None:
    config_path = tmp_path / "malformed.json"
    config_path.write_text("{not valid JSON", encoding="utf-8")

    with pytest.raises(SafetyConfigError, match="Invalid JSON"):
        load_safety_limits(config_path)


def test_missing_safety_field_is_rejected(tmp_path: Path) -> None:
    payload = _payload()
    del payload["max_abs_joint_velocity_rad_s"]
    config_path = tmp_path / "missing.json"
    _write_payload(config_path, payload)

    with pytest.raises(SafetyConfigError, match="missing required field"):
        load_safety_limits(config_path)


def test_unknown_safety_field_is_rejected(tmp_path: Path) -> None:
    payload = _payload()
    payload["torque_rate_limit_n_m_s"] = 10.0
    config_path = tmp_path / "unknown.json"
    _write_payload(config_path, payload)

    with pytest.raises(SafetyConfigError, match="unsupported field"):
        load_safety_limits(config_path)


def test_invalid_safety_value_is_rejected(tmp_path: Path) -> None:
    payload = _payload()
    payload["min_joint_angle_rad"] = 1.0
    config_path = tmp_path / "invalid.json"
    _write_payload(config_path, payload)

    with pytest.raises(SafetyConfigError, match="min_joint_angle_rad"):
        load_safety_limits(config_path)


def test_unsupported_safety_schema_is_rejected(tmp_path: Path) -> None:
    payload = _payload()
    payload["schema_version"] = 2
    config_path = tmp_path / "unsupported-schema.json"
    _write_payload(config_path, payload)

    with pytest.raises(SafetyConfigError, match="schema_version"):
        load_safety_limits(config_path)


def test_non_numeric_safety_value_is_rejected(tmp_path: Path) -> None:
    payload = _payload()
    payload["max_abs_assistive_torque_n_m"] = "2.0"
    config_path = tmp_path / "non-numeric.json"
    _write_payload(config_path, payload)

    with pytest.raises(SafetyConfigError, match="must be a JSON number"):
        load_safety_limits(config_path)
