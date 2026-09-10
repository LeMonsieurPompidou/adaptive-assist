"""Tests for strict impedance-controller configuration loading."""

import json
import math
from pathlib import Path
from typing import cast

import pytest

from adaptive_assist.controllers import (
    IMPEDANCE_CONFIG_SCHEMA_VERSION,
    ImpedanceConfigError,
    load_impedance_controller_parameters,
)

REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
CONFIG_PATH = REPOSITORY_ROOT / "configs/controllers/impedance_baseline.json"


def _payload() -> dict[str, object]:
    raw_payload: object = json.loads(CONFIG_PATH.read_text(encoding="utf-8"))
    return cast(dict[str, object], raw_payload)


def _write_payload(path: Path, payload: dict[str, object]) -> None:
    path.write_text(json.dumps(payload), encoding="utf-8")


def test_valid_impedance_configuration_loads() -> None:
    parameters = load_impedance_controller_parameters(CONFIG_PATH)

    assert IMPEDANCE_CONFIG_SCHEMA_VERSION == 1
    assert parameters.proportional_gain_n_m_per_rad == pytest.approx(20.0)
    assert parameters.derivative_gain_n_m_s_per_rad == pytest.approx(4.0)


def test_missing_configuration_field_is_rejected(tmp_path: Path) -> None:
    payload = _payload()
    del payload["proportional_gain_n_m_per_rad"]
    config_path = tmp_path / "missing-field.json"
    _write_payload(config_path, payload)

    with pytest.raises(ImpedanceConfigError, match="missing required field"):
        load_impedance_controller_parameters(config_path)


def test_unknown_configuration_field_is_rejected(tmp_path: Path) -> None:
    payload = _payload()
    payload["torque_limit_n_m"] = 5.0
    config_path = tmp_path / "unknown-field.json"
    _write_payload(config_path, payload)

    with pytest.raises(ImpedanceConfigError, match="unsupported field"):
        load_impedance_controller_parameters(config_path)


@pytest.mark.parametrize(
    ("field", "value"),
    [
        ("proportional_gain_n_m_per_rad", -1.0),
        ("proportional_gain_n_m_per_rad", math.inf),
        ("derivative_gain_n_m_s_per_rad", -1.0),
        ("derivative_gain_n_m_s_per_rad", math.nan),
    ],
)
def test_invalid_configured_gain_is_rejected(
    tmp_path: Path,
    field: str,
    value: float,
) -> None:
    payload = _payload()
    payload[field] = value
    config_path = tmp_path / "invalid-gain.json"
    _write_payload(config_path, payload)

    with pytest.raises(ImpedanceConfigError, match=field):
        load_impedance_controller_parameters(config_path)
