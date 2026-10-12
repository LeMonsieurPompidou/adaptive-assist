"""Run deterministic frequency, inertia, and disturbance evaluation."""

import argparse
from pathlib import Path

from adaptive_assist.controllers import (
    load_computed_torque_controller_parameters,
    load_impedance_controller_parameters,
)
from adaptive_assist.evaluation import (
    ExcitationRunResult,
    ExcitationSweepResult,
    RobustnessController,
    load_excitation_sweep_config,
    run_excitation_sweep,
    summarize_controller_excitation,
    write_excitation_summary_csv,
)
from adaptive_assist.experiments import load_scenario
from adaptive_assist.safety import load_safety_limits

CONFIG_PATH = Path("configs/robustness/excitation_disturbance_sweep.json")


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Run deterministic excitation and disturbance evaluation."
    )
    parser.add_argument(
        "--unsupervised",
        action="store_true",
        help="disable the configured supervisor to inspect raw controller behavior",
    )
    parser.add_argument(
        "--csv",
        type=Path,
        metavar="PATH",
        help="write summary rows to an explicit CSV path",
    )
    return parser.parse_args()


def load_and_run_excitation_sweep(
    repository_root: Path,
    *,
    unsupervised: bool,
) -> ExcitationSweepResult:
    """Load version-controlled inputs and execute one evaluation mode."""
    configuration = load_excitation_sweep_config(repository_root / CONFIG_PATH)
    scenario = load_scenario(repository_root / configuration.scenario_config_path)
    safety_limits = (
        load_safety_limits(repository_root / configuration.safety_limits_config_path)
        if configuration.safety_supervision_enabled and not unsupervised
        else None
    )
    return run_excitation_sweep(
        configuration,
        scenario,
        load_impedance_controller_parameters(
            repository_root / configuration.impedance_controller_config_path
        ),
        load_computed_torque_controller_parameters(
            repository_root / configuration.computed_torque_controller_config_path
        ),
        safety_limits,
    )


def _recovery_label(run: ExcitationRunResult) -> str:
    if not run.recovery_applicable:
        return "n/a"
    if run.recovery_time_s is None:
        return "not recovered"
    return f"{run.recovery_time_s:.3f} s"


def _print_controller_table(
    result: ExcitationSweepResult,
    controller_name: RobustnessController,
) -> None:
    print(f"\nController: {controller_name.value}")
    print(
        "Frequency  Inertia  Disturbance      RMSE(rad)  Peak error  "
        "Recovery       Interventions"
    )
    print("-" * 94)
    for run in result.runs_for_controller(controller_name):
        print(
            f"{run.case.frequency_hz:>8.2f}  "
            f"{run.case.inertia_scale_factor:>7.2f}  "
            f"{run.case.disturbance_profile.value:<16}  "
            f"{run.tracking_rmse_rad:>9.6f}  "
            f"{run.peak_tracking_error_rad:>10.6f}  "
            f"{_recovery_label(run):<13}  "
            f"{run.intervention_count:>13d}"
        )

    summary = summarize_controller_excitation(result, controller_name)
    longest = summary.longest_successful_recovery_run
    print(f"Worst RMSE: {summary.worst_rmse_run.tracking_rmse_rad:.6f} rad")
    print(f"  case: {summary.worst_rmse_run.case.case_name}")
    print(
        "Worst peak error: "
        f"{summary.worst_peak_error_run.peak_tracking_error_rad:.6f} rad"
    )
    print(f"  case: {summary.worst_peak_error_run.case.case_name}")
    print(
        "Longest successful recovery: "
        + (
            "none"
            if longest is None
            else f"{longest.recovery_time_s:.3f} s - {longest.case.case_name}"
        )
    )
    print(f"Failed pulse recoveries: {len(summary.failed_recovery_runs)}")
    if summary.most_interventions_run.intervention_count == 0:
        print("Most interventions: 0 (none)")
    else:
        print(
            "Most interventions: "
            f"{summary.most_interventions_run.intervention_count} - "
            f"{summary.most_interventions_run.case.case_name}"
        )


def main() -> int:
    """Run the configured evaluation and print deterministic summaries."""
    args = _parse_args()
    repository_root = Path(__file__).resolve().parents[1]
    result = load_and_run_excitation_sweep(
        repository_root,
        unsupervised=args.unsupervised,
    )

    print("Excitation robustness evaluation")
    print(
        "Safety supervision: "
        + ("enabled" if result.supervision_active else "disabled")
    )
    print("Engineering mathematical-simulation study; not real-world validation.")
    for controller_name in RobustnessController:
        _print_controller_table(result, controller_name)

    if args.csv is not None:
        output_path = write_excitation_summary_csv(result, args.csv)
        print(f"\nCSV: {output_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
