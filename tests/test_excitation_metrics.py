"""Tests for excitation-specific tracking and pulse-recovery metrics."""

import math

import pytest

from adaptive_assist import JointState, JointTorques
from adaptive_assist.evaluation import disturbance_recovery_time_s
from adaptive_assist.experiments import (
    ExperimentMetadata,
    ExperimentResult,
    ExperimentSample,
    JointReference,
    peak_absolute_tracking_error_rad,
)


def _result(samples: tuple[tuple[float, float], ...]) -> ExperimentResult:
    return ExperimentResult(
        scenario_name="synthetic_recovery",
        samples=tuple(
            ExperimentSample(
                time_s=time_s,
                actual_state=JointState(angle_error_rad, 0.0),
                reference=JointReference(0.0, 0.0, 0.0),
                requested_assistive_torque_n_m=0.0,
                applied_torques=JointTorques(),
                angular_acceleration_rad_s2=0.0,
            )
            for time_s, angle_error_rad in samples
        ),
        metadata=ExperimentMetadata(
            scenario_schema_version=1,
            integrator_name="synthetic",
            reference_type="ConstantReference",
            duration_s=samples[-1][0],
            time_step_s=0.1,
            integration_steps=len(samples) - 1,
        ),
    )


def _recovery(result: ExperimentResult) -> float | None:
    return disturbance_recovery_time_s(
        result,
        pulse_end_time_s=0.2,
        tolerance_rad=0.05,
        dwell_duration_s=0.2,
    )


def test_peak_absolute_tracking_error() -> None:
    result = _result(((0.0, -0.1), (0.1, 0.25), (0.2, -0.2)))

    assert peak_absolute_tracking_error_rad(result) == pytest.approx(0.25)


def test_immediate_recovery() -> None:
    result = _result(((0.0, 0.2), (0.1, 0.1), (0.2, 0.05), (0.3, 0.04), (0.4, 0.03)))

    assert _recovery(result) == pytest.approx(0.0)


def test_delayed_recovery() -> None:
    result = _result(
        ((0.0, 0.2), (0.1, 0.1), (0.2, 0.08), (0.3, 0.04), (0.4, 0.03), (0.5, 0.02))
    )

    assert _recovery(result) == pytest.approx(0.1)


def test_temporary_tolerance_entry_does_not_satisfy_dwell() -> None:
    result = _result(
        (
            (0.0, 0.2),
            (0.1, 0.1),
            (0.2, 0.04),
            (0.3, 0.06),
            (0.4, 0.04),
            (0.5, 0.03),
            (0.6, 0.02),
        )
    )

    assert _recovery(result) == pytest.approx(0.2)


def test_no_recovery_is_explicit() -> None:
    result = _result(((0.0, 0.2), (0.1, 0.1), (0.2, 0.08), (0.3, 0.07), (0.4, 0.06)))

    assert _recovery(result) is None


def test_exact_dwell_boundary_counts_as_recovered() -> None:
    result = _result(((0.0, 0.2), (0.1, 0.1), (0.2, 0.05), (0.3, 0.05), (0.4, 0.05)))

    assert _recovery(result) == pytest.approx(0.0)


def test_pulse_ending_near_experiment_end_cannot_complete_dwell() -> None:
    result = _result(((0.8, 0.03), (0.9, 0.03), (1.0, 0.03)))

    recovery = disturbance_recovery_time_s(
        result,
        pulse_end_time_s=0.9,
        tolerance_rad=0.05,
        dwell_duration_s=0.2,
    )

    assert recovery is None


def test_recovery_metric_rejects_empty_results() -> None:
    empty_result = ExperimentResult(
        scenario_name="empty",
        samples=(),
        metadata=ExperimentMetadata(1, "synthetic", "synthetic", 0.0, 0.1, 0),
    )

    with pytest.raises(ValueError, match="at least one sample"):
        disturbance_recovery_time_s(
            empty_result,
            pulse_end_time_s=0.2,
            tolerance_rad=0.05,
            dwell_duration_s=0.2,
        )


@pytest.mark.parametrize(
    ("pulse_end_time_s", "tolerance_rad", "dwell_duration_s"),
    [
        (-0.1, 0.05, 0.2),
        (0.2, 0.0, 0.2),
        (0.2, 0.05, 0.0),
        (math.inf, 0.05, 0.2),
    ],
)
def test_invalid_recovery_parameters_are_rejected(
    pulse_end_time_s: float,
    tolerance_rad: float,
    dwell_duration_s: float,
) -> None:
    with pytest.raises(ValueError):
        disturbance_recovery_time_s(
            _result(((0.0, 0.0),)),
            pulse_end_time_s=pulse_end_time_s,
            tolerance_rad=tolerance_rad,
            dwell_duration_s=dwell_duration_s,
        )
