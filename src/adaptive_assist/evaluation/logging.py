"""Standard-library CSV export for robustness summaries."""

import csv
from pathlib import Path

from adaptive_assist.evaluation.robustness import RobustnessSweepResult

ROBUSTNESS_CSV_COLUMNS = (
    "controller",
    "supervision_active",
    "case_name",
    "varied_parameter",
    "scale_factor",
    "tracking_rmse_rad",
    "rmse_degradation_fraction",
    "peak_requested_assistive_torque_n_m",
    "peak_applied_assistive_torque_n_m",
    "intervention_count",
    "intervention_fraction",
    "maximum_torque_modification_n_m",
    "actual_inertia_kg_m2",
    "actual_mass_kg",
    "actual_center_of_mass_distance_m",
    "actual_damping_n_m_s_per_rad",
    "actual_stiffness_n_m_per_rad",
)


def write_robustness_summary_csv(
    result: RobustnessSweepResult,
    output_path: str | Path,
) -> Path:
    """Write deterministically ordered robustness summaries to CSV."""
    destination = Path(output_path)
    with destination.open("w", encoding="utf-8", newline="") as csv_file:
        writer = csv.writer(csv_file)
        writer.writerow(ROBUSTNESS_CSV_COLUMNS)
        for run in result.runs:
            actual = run.actual_plant_parameters
            writer.writerow(
                (
                    run.controller_name.value,
                    result.supervision_active,
                    run.case.case_name,
                    run.case.parameter_label,
                    "" if run.case.scale_factor is None else run.case.scale_factor,
                    run.tracking_rmse_rad,
                    run.rmse_degradation_fraction,
                    run.peak_requested_assistive_torque_n_m,
                    run.peak_applied_assistive_torque_n_m,
                    run.intervention_count,
                    run.intervention_fraction,
                    run.maximum_torque_modification_n_m,
                    actual.inertia_kg_m2,
                    actual.mass_kg,
                    actual.center_of_mass_distance_m,
                    actual.damping_n_m_s_per_rad,
                    actual.stiffness_n_m_per_rad,
                )
            )
    return destination
