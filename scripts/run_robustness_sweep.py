"""Run the configured deterministic plant/model mismatch sweep."""

import argparse
from pathlib import Path

from adaptive_assist.controllers import (
    load_computed_torque_controller_parameters,
    load_impedance_controller_parameters,
)
from adaptive_assist.evaluation import (
    MismatchParameter,
    RobustnessController,
    RobustnessRunResult,
    RobustnessSweepResult,
    load_robustness_sweep_config,
    run_robustness_sweep,
    summarize_controller_robustness,
    write_robustness_summary_csv,
)
from adaptive_assist.experiments import load_scenario
from adaptive_assist.safety import load_safety_limits

CONFIG_PATH = Path("configs/robustness/model_mismatch_sweep.json")

PARAMETER_LABELS = {
    MismatchParameter.INERTIA: "inertia",
    MismatchParameter.MASS: "mass",
    MismatchParameter.CENTER_OF_MASS_DISTANCE: "COM distance",
    MismatchParameter.DAMPING: "damping",
    MismatchParameter.STIFFNESS: "stiffness",
}


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Run deterministic controller robustness evaluation."
    )
    parser.add_argument(
        "--unsupervised",
        action="store_true",
        help="disable the configured supervisor to expose raw controller behavior",
    )
    parser.add_argument(
        "--csv",
        type=Path,
        metavar="PATH",
        help="write summary rows to an explicit CSV path",
    )
    return parser.parse_args()


def load_and_run_sweep(
    repository_root: Path,
    *,
    unsupervised: bool,
) -> RobustnessSweepResult:
    """Load version-controlled inputs and run one deterministic sweep."""
    configuration = load_robustness_sweep_config(repository_root / CONFIG_PATH)
    scenario = load_scenario(repository_root / configuration.scenario_config_path)
    impedance_parameters = load_impedance_controller_parameters(
        repository_root / configuration.impedance_controller_config_path
    )
    computed_torque_parameters = load_computed_torque_controller_parameters(
        repository_root / configuration.computed_torque_controller_config_path
    )
    supervision_active = configuration.safety_supervision_enabled and not unsupervised
    safety_limits = (
        load_safety_limits(repository_root / configuration.safety_limits_config_path)
        if supervision_active
        else None
    )
    return run_robustness_sweep(
        configuration,
        scenario,
        impedance_parameters,
        computed_torque_parameters,
        safety_limits,
    )


def _run_for_parameter_scale(
    result: RobustnessSweepResult,
    controller_name: RobustnessController,
    parameter: MismatchParameter,
    scale_factor: float,
) -> RobustnessRunResult:
    if scale_factor == 1.0:
        return result.nominal_run(controller_name)
    for run in result.runs_for_controller(controller_name):
        if (
            run.case.varied_parameter is parameter
            and run.case.scale_factor == scale_factor
        ):
            return run
    raise ValueError(
        f"missing robustness run for {parameter.value} at {scale_factor:g}x"
    )


def _print_run_row(
    parameter_label: str,
    scale_label: str,
    run: RobustnessRunResult,
) -> None:
    print(
        f"{parameter_label:<17}"
        f"{scale_label:>7}"
        f"{run.tracking_rmse_rad:>13.6f}"
        f"{run.rmse_degradation_fraction:>12.6f}"
        f"{run.peak_applied_assistive_torque_n_m:>17.6f}"
        f"{run.intervention_count:>16d}"
    )


def _print_controller_table(
    result: RobustnessSweepResult,
    controller_name: RobustnessController,
) -> None:
    print()
    print(f"Controller: {controller_name.value}")
    print(
        f"{'Parameter':<17}"
        f"{'Scale':>7}"
        f"{'RMSE (rad)':>13}"
        f"{'RMSE delta':>12}"
        f"{'Peak applied':>17}"
        f"{'Interventions':>16}"
    )
    print("-" * 82)
    for parameter in result.configuration.varied_parameters:
        for scale_factor in result.configuration.scale_factors:
            run = _run_for_parameter_scale(
                result,
                controller_name,
                parameter,
                scale_factor,
            )
            _print_run_row(
                PARAMETER_LABELS[parameter],
                f"{scale_factor:.2f}",
                run,
            )
    combined_run = next(
        run
        for run in result.runs_for_controller(controller_name)
        if run.case.case_name == result.configuration.combined_mismatch.case_name
    )
    _print_run_row("combined", "-", combined_run)


def main() -> int:
    """Run, report, and optionally export the configured robustness sweep."""
    args = _parse_args()
    repository_root = Path(__file__).resolve().parents[1]
    result = load_and_run_sweep(
        repository_root,
        unsupervised=args.unsupervised,
    )

    print("Model-mismatch robustness evaluation")
    print(
        "Deterministic engineering simulation only; mismatch ranges are "
        "illustrative, not identified human or hardware uncertainty"
    )
    print(
        f"Safety supervision: {'enabled' if result.supervision_active else 'disabled'}"
    )
    for controller_name in RobustnessController:
        _print_controller_table(result, controller_name)

    print()
    print("Worst RMSE in configured sweep")
    for controller_name in RobustnessController:
        summary = summarize_controller_robustness(result, controller_name)
        print(
            f"{controller_name.value:<17}"
            f"{summary.worst_case_rmse_rad:.6f} rad -- "
            f"{summary.worst_case_name} "
            f"(degradation {summary.worst_case_rmse_degradation_fraction:.6f})"
        )
    print("These results describe only this configured deterministic sweep.")

    if args.csv is not None:
        output_path = write_robustness_summary_csv(result, args.csv)
        print(f"CSV: {output_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
