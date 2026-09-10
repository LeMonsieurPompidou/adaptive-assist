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
    """Return maximum absolute recorded assistive torque in newton metres."""
    samples = _require_samples(result)
    return max(abs(sample.applied_torques.assistive_torque_n_m) for sample in samples)


def _require_samples(result: ExperimentResult) -> tuple[ExperimentSample, ...]:
    if not result.samples:
        raise ValueError("experiment result must contain at least one sample")
    return result.samples
