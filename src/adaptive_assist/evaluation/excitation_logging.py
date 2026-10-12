"""Standard-library CSV export for excitation robustness summaries."""

import csv
from pathlib import Path

from adaptive_assist.evaluation.excitation import ExcitationSweepResult

EXCITATION_CSV_COLUMNS = (
    "controller",
    "supervision_active",
    "case_name",
    "frequency_hz",
    "inertia_scale",
    "disturbance_profile",
    "tracking_rmse_rad",
    "peak_tracking_error_rad",
    "recovery_time_s",
    "recovered",
    "peak_requested_assistive_torque_n_m",
    "peak_applied_assistive_torque_n_m",
    "intervention_count",
    "intervention_fraction",
    "maximum_torque_modification_n_m",
    "actual_inertia_kg_m2",
    "nominal_controller_inertia_kg_m2",
)


def write_excitation_summary_csv(
    result: ExcitationSweepResult,
    output_path: str | Path,
) -> Path:
    """Write deterministically ordered excitation summaries to CSV."""
    destination = Path(output_path)
    with destination.open("w", encoding="utf-8", newline="") as csv_file:
        writer = csv.writer(csv_file)
        writer.writerow(EXCITATION_CSV_COLUMNS)
        for run in result.runs:
            writer.writerow(
                (
                    run.controller_name.value,
                    result.supervision_active,
                    run.case.case_name,
                    run.case.frequency_hz,
                    run.case.inertia_scale_factor,
                    run.case.disturbance_profile.value,
                    run.tracking_rmse_rad,
                    run.peak_tracking_error_rad,
                    "" if run.recovery_time_s is None else run.recovery_time_s,
                    "" if run.recovered is None else run.recovered,
                    run.peak_requested_assistive_torque_n_m,
                    run.peak_applied_assistive_torque_n_m,
                    run.intervention_count,
                    run.intervention_fraction,
                    run.maximum_torque_modification_n_m,
                    run.actual_plant_parameters.inertia_kg_m2,
                    (
                        ""
                        if run.nominal_controller_model_parameters is None
                        else run.nominal_controller_model_parameters.inertia_kg_m2
                    ),
                )
            )
    return destination
