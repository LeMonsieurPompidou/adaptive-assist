"""Tests for strict, versioned experiment scenario loading."""

import json
import math
from pathlib import Path
from typing import cast

import pytest

from adaptive_assist.experiments import (
    SCENARIO_SCHEMA_VERSION,
    ConstantReference,
    ScenarioConfigError,
    SinusoidalReference,
    load_scenario,
)

REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
NOMINAL_SCENARIO_PATH = REPOSITORY_ROOT / "configs/scenarios/nominal_open_loop.json"


def _nominal_payload() -> dict[str, object]:
    raw_payload: object = json.loads(NOMINAL_SCENARIO_PATH.read_text(encoding="utf-8"))
    return cast(dict[str, object], raw_payload)


def _write_payload(path: Path, payload: dict[str, object]) -> None:
    path.write_text(json.dumps(payload), encoding="utf-8")


def test_nominal_json_configuration_loads() -> None:
    scenario = load_scenario(NOMINAL_SCENARIO_PATH)

    assert scenario.schema_version == SCENARIO_SCHEMA_VERSION
    assert scenario.scenario_name == "nominal_open_loop"
    assert scenario.step_count == 100
    assert scenario.initial_state.angle_rad == 0.0
    assert scenario.joint_parameters.inertia_kg_m2 == 1.2
    assert scenario.torques.assistive_torque_n_m == 1.0
    assert isinstance(scenario.reference, SinusoidalReference)


def test_constant_reference_configuration_loads(tmp_path: Path) -> None:
    payload = _nominal_payload()
    payload["reference"] = {
        "type": "constant",
        "angle_rad": 0.25,
        "angular_velocity_rad_s": 0.0,
        "angular_acceleration_rad_s2": 0.0,
    }
    scenario_path = tmp_path / "constant-reference.json"
    _write_payload(scenario_path, payload)

    scenario = load_scenario(scenario_path)

    assert isinstance(scenario.reference, ConstantReference)
    assert scenario.reference.evaluate(0.5).angle_rad == pytest.approx(0.25)


def test_malformed_json_is_rejected(tmp_path: Path) -> None:
    scenario_path = tmp_path / "malformed.json"
    scenario_path.write_text("{not valid JSON", encoding="utf-8")

    with pytest.raises(ScenarioConfigError, match="Invalid JSON"):
        load_scenario(scenario_path)


@pytest.mark.parametrize(
    ("field", "value"),
    [
        ("duration_s", 0.0),
        ("duration_s", -1.0),
        ("duration_s", math.inf),
        ("time_step_s", 0.0),
        ("time_step_s", -0.1),
        ("time_step_s", math.nan),
    ],
)
def test_invalid_scenario_timing_is_rejected(
    tmp_path: Path,
    field: str,
    value: float,
) -> None:
    payload = _nominal_payload()
    payload[field] = value
    scenario_path = tmp_path / "invalid-timing.json"
    _write_payload(scenario_path, payload)

    with pytest.raises(ScenarioConfigError, match=field):
        load_scenario(scenario_path)


def test_duration_must_be_compatible_with_fixed_step(tmp_path: Path) -> None:
    payload = _nominal_payload()
    payload["duration_s"] = 0.15
    payload["time_step_s"] = 0.1
    scenario_path = tmp_path / "incompatible-duration.json"
    _write_payload(scenario_path, payload)

    with pytest.raises(ScenarioConfigError, match="integer multiple"):
        load_scenario(scenario_path)


def test_invalid_physical_parameters_use_plant_validation(tmp_path: Path) -> None:
    payload = _nominal_payload()
    parameters = cast(dict[str, object], payload["joint_parameters"])
    parameters["inertia_kg_m2"] = 0.0
    scenario_path = tmp_path / "invalid-parameters.json"
    _write_payload(scenario_path, payload)

    with pytest.raises(ScenarioConfigError, match="inertia_kg_m2") as error_info:
        load_scenario(scenario_path)

    assert isinstance(error_info.value.__cause__, ValueError)


def test_unsupported_reference_type_is_rejected(tmp_path: Path) -> None:
    payload = _nominal_payload()
    reference = cast(dict[str, object], payload["reference"])
    reference["type"] = "random"
    scenario_path = tmp_path / "unsupported-reference.json"
    _write_payload(scenario_path, payload)

    with pytest.raises(ScenarioConfigError, match="reference.type"):
        load_scenario(scenario_path)


def test_unsupported_schema_version_is_rejected(tmp_path: Path) -> None:
    payload = _nominal_payload()
    payload["schema_version"] = 2
    scenario_path = tmp_path / "unsupported-schema.json"
    _write_payload(scenario_path, payload)

    with pytest.raises(ScenarioConfigError, match="schema_version"):
        load_scenario(scenario_path)


def test_unsupported_fields_are_rejected(tmp_path: Path) -> None:
    payload = _nominal_payload()
    payload["controller"] = "not-supported"
    scenario_path = tmp_path / "unsupported-field.json"
    _write_payload(scenario_path, payload)

    with pytest.raises(ScenarioConfigError, match="unsupported field"):
        load_scenario(scenario_path)
