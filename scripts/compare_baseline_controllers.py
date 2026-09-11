"""Compare open-loop, impedance, and computed-torque assistance."""

from pathlib import Path

from adaptive_assist import OneDofJointModel
from adaptive_assist.controllers import (
    ComputedTorqueController,
    ImpedanceController,
    load_computed_torque_controller_parameters,
    load_impedance_controller_parameters,
)
from adaptive_assist.experiments import (
    ExperimentResult,
    load_scenario,
    peak_assistive_torque_n_m,
    run_closed_loop_experiment,
    run_open_loop_experiment,
    trajectory_tracking_rmse_rad,
)


def run_comparison() -> tuple[ExperimentResult, ExperimentResult, ExperimentResult]:
    """Run all three strategies under one immutable scenario definition."""
    repository_root = Path(__file__).resolve().parents[1]
    scenario = load_scenario(
        repository_root / "configs/scenarios/nominal_tracking.json"
    )
    impedance_parameters = load_impedance_controller_parameters(
        repository_root / "configs/controllers/impedance_baseline.json"
    )
    computed_torque_parameters = load_computed_torque_controller_parameters(
        repository_root / "configs/controllers/computed_torque_baseline.json"
    )

    open_loop_result = run_open_loop_experiment(
        scenario,
        OneDofJointModel(scenario.joint_parameters),
    )
    impedance_result = run_closed_loop_experiment(
        scenario,
        OneDofJointModel(scenario.joint_parameters),
        ImpedanceController(impedance_parameters),
    )
    computed_torque_result = run_closed_loop_experiment(
        scenario,
        OneDofJointModel(scenario.joint_parameters),
        ComputedTorqueController(
            parameters=computed_torque_parameters,
            nominal_model=OneDofJointModel(scenario.joint_parameters),
        ),
    )
    return open_loop_result, impedance_result, computed_torque_result


def _metric_row(
    label: str,
    open_loop_value: float,
    impedance_value: float,
    computed_torque_value: float,
) -> str:
    return (
        f"{label:<28}"
        f"{open_loop_value:>14.6f}"
        f"{impedance_value:>14.6f}"
        f"{computed_torque_value:>19.6f}"
    )


def main() -> int:
    """Print controller-independent metrics for all baseline runs."""
    open_loop_result, impedance_result, computed_torque_result = run_comparison()

    print("Baseline controller comparison")
    print(
        "Engineering simulation only; not a safety benchmark or biomechanical "
        "or clinical validation"
    )
    print("-" * 75)
    print(f"{'Metric':<28}{'Open loop':>14}{'Impedance':>14}{'Computed torque':>19}")
    print(
        _metric_row(
            "Tracking RMSE (rad)",
            trajectory_tracking_rmse_rad(open_loop_result),
            trajectory_tracking_rmse_rad(impedance_result),
            trajectory_tracking_rmse_rad(computed_torque_result),
        )
    )
    print(
        _metric_row(
            "Peak assist torque (N m)",
            peak_assistive_torque_n_m(open_loop_result),
            peak_assistive_torque_n_m(impedance_result),
            peak_assistive_torque_n_m(computed_torque_result),
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
