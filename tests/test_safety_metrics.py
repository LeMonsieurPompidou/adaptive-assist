"""Tests for controller-independent safety-intervention metrics."""

import math
from collections.abc import Callable

import pytest

from adaptive_assist import JointState, JointTorques
from adaptive_assist.experiments import (
    ExperimentMetadata,
    ExperimentResult,
    ExperimentSample,
    JointReference,
    maximum_torque_modification_n_m,
    peak_assistive_torque_n_m,
    peak_requested_assistive_torque_n_m,
    safety_intervention_count,
    safety_intervention_fraction,
)
from adaptive_assist.safety import SafetyInterventionReason


def _result(
    requested_torques_n_m: tuple[float, ...],
    applied_torques_n_m: tuple[float, ...],
    intervention_indices: frozenset[int] = frozenset(),
) -> ExperimentResult:
    samples = tuple(
        ExperimentSample(
            time_s=float(index),
            actual_state=JointState(0.0, 0.0),
            reference=JointReference(0.0, 0.0, 0.0),
            requested_assistive_torque_n_m=requested_torque,
            applied_torques=JointTorques(assistive_torque_n_m=applied_torque),
            angular_acceleration_rad_s2=0.0,
            safety_intervention_reasons=(
                (SafetyInterventionReason.TORQUE_LIMIT,)
                if index in intervention_indices
                else ()
            ),
        )
        for index, (requested_torque, applied_torque) in enumerate(
            zip(requested_torques_n_m, applied_torques_n_m, strict=True)
        )
    )
    return ExperimentResult(
        scenario_name="safety_metric_test",
        samples=samples,
        metadata=ExperimentMetadata(
            scenario_schema_version=1,
            integrator_name="synthetic",
            reference_type="synthetic",
            duration_s=float(max(0, len(samples) - 1)),
            time_step_s=1.0,
            integration_steps=max(0, len(samples) - 1),
            safety_supervision_active=True,
        ),
    )


def test_safety_metrics_use_requested_and_applied_commands() -> None:
    result = _result(
        requested_torques_n_m=(1.0, 5.0, -6.0, 2.0),
        applied_torques_n_m=(1.0, 3.0, -3.0, 2.0),
        intervention_indices=frozenset({1, 2}),
    )

    assert safety_intervention_count(result) == 2
    assert safety_intervention_fraction(result) == pytest.approx(0.5)
    assert maximum_torque_modification_n_m(result) == pytest.approx(3.0)
    assert peak_requested_assistive_torque_n_m(result) == pytest.approx(6.0)
    assert peak_assistive_torque_n_m(result) == pytest.approx(3.0)


def test_no_intervention_has_zero_count_fraction_and_modification() -> None:
    result = _result((1.0, -2.0), (1.0, -2.0))

    assert safety_intervention_count(result) == 0
    assert safety_intervention_fraction(result) == pytest.approx(0.0)
    assert maximum_torque_modification_n_m(result) == pytest.approx(0.0)


def test_non_finite_requested_command_has_infinite_magnitude_metrics() -> None:
    sample = ExperimentSample(
        time_s=0.0,
        actual_state=JointState(0.0, 0.0),
        reference=JointReference(0.0, 0.0, 0.0),
        requested_assistive_torque_n_m=math.nan,
        applied_torques=JointTorques(assistive_torque_n_m=0.0),
        angular_acceleration_rad_s2=0.0,
        safety_intervention_reasons=(
            SafetyInterventionReason.INVALID_REQUESTED_COMMAND,
        ),
    )
    result = ExperimentResult(
        scenario_name="invalid_command_metric_test",
        samples=(sample,),
        metadata=ExperimentMetadata(
            scenario_schema_version=1,
            integrator_name="synthetic",
            reference_type="synthetic",
            duration_s=0.0,
            time_step_s=1.0,
            integration_steps=0,
            safety_supervision_active=True,
        ),
    )

    assert math.isinf(peak_requested_assistive_torque_n_m(result))
    assert math.isinf(maximum_torque_modification_n_m(result))


@pytest.mark.parametrize(
    "metric",
    [
        safety_intervention_count,
        safety_intervention_fraction,
        maximum_torque_modification_n_m,
        peak_requested_assistive_torque_n_m,
    ],
)
def test_safety_metrics_reject_empty_results(
    metric: Callable[[ExperimentResult], float | int],
) -> None:
    with pytest.raises(ValueError, match="at least one sample"):
        metric(_result((), ()))
