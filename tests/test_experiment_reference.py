"""Tests for deterministic experiment reference signals."""

import math

import pytest

from adaptive_assist.experiments import (
    ConstantReference,
    JointReference,
    SinusoidalReference,
)


def test_constant_reference_is_constant() -> None:
    expected = JointReference(
        angle_rad=0.3,
        angular_velocity_rad_s=-0.2,
        angular_acceleration_rad_s2=0.1,
    )
    signal = ConstantReference(expected)

    assert signal.evaluate(0.0) == expected
    assert signal.evaluate(12.5) == expected


def test_sinusoidal_reference_is_analytically_consistent() -> None:
    signal = SinusoidalReference(
        amplitude_rad=2.0,
        frequency_hz=0.5,
        offset_rad=0.25,
    )

    at_zero = signal.evaluate(0.0)
    at_quarter_period = signal.evaluate(0.5)

    assert at_zero.angle_rad == pytest.approx(0.25)
    assert at_zero.angular_velocity_rad_s == pytest.approx(2.0 * math.pi)
    assert at_zero.angular_acceleration_rad_s2 == pytest.approx(0.0)
    assert at_quarter_period.angle_rad == pytest.approx(2.25)
    assert at_quarter_period.angular_velocity_rad_s == pytest.approx(0.0, abs=1e-12)
    assert at_quarter_period.angular_acceleration_rad_s2 == pytest.approx(
        -2.0 * math.pi**2
    )


def test_identical_reference_inputs_produce_identical_values() -> None:
    signal = SinusoidalReference(
        amplitude_rad=0.4,
        frequency_hz=1.2,
        offset_rad=-0.1,
    )

    assert signal.evaluate(0.37) == signal.evaluate(0.37)


@pytest.mark.parametrize(
    ("field", "value"),
    [
        ("amplitude_rad", -0.1),
        ("amplitude_rad", math.inf),
        ("frequency_hz", 0.0),
        ("frequency_hz", math.nan),
        ("offset_rad", -math.inf),
    ],
)
def test_invalid_sinusoidal_parameters_are_rejected(
    field: str,
    value: float,
) -> None:
    values = {
        "amplitude_rad": 0.2,
        "frequency_hz": 1.0,
        "offset_rad": 0.0,
    }
    values[field] = value

    with pytest.raises(ValueError, match=field):
        SinusoidalReference(
            amplitude_rad=values["amplitude_rad"],
            frequency_hz=values["frequency_hz"],
            offset_rad=values["offset_rad"],
        )


@pytest.mark.parametrize("time_s", [-0.1, math.inf, math.nan])
def test_invalid_reference_times_are_rejected(time_s: float) -> None:
    signal = ConstantReference(JointReference(0.0, 0.0, 0.0))

    with pytest.raises(ValueError, match="time_s"):
        signal.evaluate(time_s)


def test_non_finite_joint_reference_is_rejected() -> None:
    with pytest.raises(ValueError, match="angular_acceleration_rad_s2"):
        JointReference(0.0, 0.0, math.nan)
