"""Controller-independent metrics computed from experiment records."""

import math

from adaptive_assist.experiments.records import ExperimentResult, ExperimentSample


def trajectory_tracking_rmse_rad(result: ExperimentResult) -> float:
    """Return joint-angle tracking root-mean-square error in radians."""
    samples = _require_samples(result)
    squared_errors = (
        (sample.actual_state.angle_rad - sample.reference.angle_rad) ** 2
        for sample in samples
    )
    return math.sqrt(math.fsum(squared_errors) / len(samples))


def peak_assistive_torque_n_m(result: ExperimentResult) -> float:
    """Return maximum absolute applied assistive torque in newton metres."""
    samples = _require_samples(result)
    return max(abs(sample.applied_torques.assistive_torque_n_m) for sample in samples)


def peak_requested_assistive_torque_n_m(result: ExperimentResult) -> float:
    """Return maximum absolute requested assistive torque in newton metres."""
    samples = _require_samples(result)
    return max(
        _absolute_requested_torque_n_m(sample.requested_assistive_torque_n_m)
        for sample in samples
    )


def safety_intervention_count(result: ExperimentResult) -> int:
    """Return the number of recorded samples with supervisor intervention."""
    samples = _require_samples(result)
    return sum(sample.safety_intervened for sample in samples)


def safety_intervention_fraction(result: ExperimentResult) -> float:
    """Return the fraction of samples with supervisor intervention."""
    samples = _require_samples(result)
    return sum(sample.safety_intervened for sample in samples) / len(samples)


def maximum_torque_modification_n_m(result: ExperimentResult) -> float:
    """Return the largest absolute requested-to-applied torque difference."""
    samples = _require_samples(result)
    return max(
        (
            math.inf
            if not math.isfinite(sample.requested_assistive_torque_n_m)
            else abs(
                sample.requested_assistive_torque_n_m
                - sample.applied_torques.assistive_torque_n_m
            )
        )
        for sample in samples
    )


def _absolute_requested_torque_n_m(requested_torque_n_m: float) -> float:
    return (
        abs(requested_torque_n_m) if math.isfinite(requested_torque_n_m) else math.inf
    )


def _require_samples(result: ExperimentResult) -> tuple[ExperimentSample, ...]:
    if not result.samples:
        raise ValueError("experiment result must contain at least one sample")
    return result.samples
