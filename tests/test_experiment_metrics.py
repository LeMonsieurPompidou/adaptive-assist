"""Tests for controller-independent experiment metrics."""

import math
from collections.abc import Callable

import pytest

from adaptive_assist import JointState, JointTorques
from adaptive_assist.experiments import (
    ExperimentMetadata,
    ExperimentResult,
    ExperimentSample,
    JointReference,
    peak_assistive_torque_n_m,
    trajectory_tracking_rmse_rad,
)


def _result(
    actual_angles_rad: tuple[float, ...],
    reference_angles_rad: tuple[float, ...],
    assistive_torques_n_m: tuple[float, ...],
) -> ExperimentResult:
    samples = tuple(
        ExperimentSample(
            time_s=float(index),
            actual_state=JointState(actual_angle, 0.0),
            reference=JointReference(reference_angle, 0.0, 0.0),
            applied_torques=JointTorques(assistive_torque_n_m=assistive_torque),
            angular_acceleration_rad_s2=0.0,
        )
        for index, (actual_angle, reference_angle, assistive_torque) in enumerate(
            zip(
                actual_angles_rad,
                reference_angles_rad,
                assistive_torques_n_m,
                strict=True,
            )
        )
    )
    return ExperimentResult(
        scenario_name="metric_test",
        samples=samples,
        metadata=ExperimentMetadata(
            scenario_schema_version=1,
            integrator_name="synthetic",
            reference_type="synthetic",
            duration_s=float(max(0, len(samples) - 1)),
            time_step_s=1.0,
            integration_steps=max(0, len(samples) - 1),
        ),
    )


def test_zero_tracking_error_has_zero_rmse() -> None:
    result = _result((0.1, -0.2), (0.1, -0.2), (0.0, 0.0))

    assert trajectory_tracking_rmse_rad(result) == pytest.approx(0.0)


def test_known_tracking_errors_have_expected_rmse() -> None:
    result = _result((0.0, 3.0, 4.0), (0.0, 0.0, 0.0), (0.0, 0.0, 0.0))

    assert trajectory_tracking_rmse_rad(result) == pytest.approx(5.0 / math.sqrt(3.0))


def test_peak_assistive_torque_uses_absolute_value() -> None:
    result = _result((0.0, 0.0), (0.0, 0.0), (-3.5, 2.0))

    assert peak_assistive_torque_n_m(result) == pytest.approx(3.5)


@pytest.mark.parametrize(
    "metric",
    [trajectory_tracking_rmse_rad, peak_assistive_torque_n_m],
)
def test_metrics_reject_empty_results(
    metric: Callable[[ExperimentResult], float],
) -> None:
    empty_result = _result((), (), ())

    with pytest.raises(ValueError, match="at least one sample"):
        metric(empty_result)
