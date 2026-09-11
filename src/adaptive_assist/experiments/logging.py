"""Standard-library CSV export for experiment samples."""

import csv
from pathlib import Path

from adaptive_assist.experiments.records import ExperimentResult

CSV_COLUMNS = (
    "time_s",
    "actual_angle_rad",
    "actual_angular_velocity_rad_s",
    "reference_angle_rad",
    "reference_angular_velocity_rad_s",
    "reference_angular_acceleration_rad_s2",
    "human_torque_n_m",
    "requested_assistive_torque_n_m",
    "applied_assistive_torque_n_m",
    "disturbance_torque_n_m",
    "angular_acceleration_rad_s2",
    "safety_supervision_active",
    "safety_intervened",
    "safety_intervention_reasons",
)


def write_experiment_csv(
    result: ExperimentResult,
    output_path: str | Path,
) -> Path:
    """Write experiment samples to an explicitly requested CSV path."""
    destination = Path(output_path)
    with destination.open("w", encoding="utf-8", newline="") as csv_file:
        writer = csv.writer(csv_file)
        writer.writerow(CSV_COLUMNS)
        for sample in result.samples:
            writer.writerow(
                (
                    sample.time_s,
                    sample.actual_state.angle_rad,
                    sample.actual_state.angular_velocity_rad_s,
                    sample.reference.angle_rad,
                    sample.reference.angular_velocity_rad_s,
                    sample.reference.angular_acceleration_rad_s2,
                    sample.applied_torques.human_torque_n_m,
                    sample.requested_assistive_torque_n_m,
                    sample.applied_torques.assistive_torque_n_m,
                    sample.applied_torques.disturbance_torque_n_m,
                    sample.angular_acceleration_rad_s2,
                    result.metadata.safety_supervision_active,
                    sample.safety_intervened,
                    "|".join(
                        reason.value for reason in sample.safety_intervention_reasons
                    ),
                )
            )
    return destination
