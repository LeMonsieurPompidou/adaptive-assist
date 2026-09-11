"""Compare open-loop and impedance assistance under one scenario."""

from pathlib import Path

from adaptive_assist import OneDofJointModel
from adaptive_assist.controllers import (
    ImpedanceController,
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


def run_comparison() -> tuple[ExperimentResult, ExperimentResult]:
    """Run both assistive-torque strategies with one immutable scenario."""
    repository_root = Path(__file__).resolve().parents[1]
    scenario = load_scenario(
        repository_root / "configs/scenarios/nominal_tracking.json"
    )
    controller_parameters = load_impedance_controller_parameters(
        repository_root / "configs/controllers/impedance_baseline.json"
    )
    open_loop_result = run_open_loop_experiment(
        scenario,
        OneDofJointModel(scenario.joint_parameters),
    )
    impedance_result = run_closed_loop_experiment(
        scenario,
        OneDofJointModel(scenario.joint_parameters),
        ImpedanceController(controller_parameters),
    )
    return open_loop_result, impedance_result


def main() -> int:
    """Print controller-independent metrics for both deterministic runs."""
    open_loop_result, impedance_result = run_comparison()

    print("Controller comparison")
    print(
        "Engineering simulation baseline; not a safety benchmark or "
        "biomechanical or clinical validation"
    )
    print("-" * 61)
    print(f"{'Metric':<28}{'Open loop':>16}{'Impedance':>17}")
    print(
        f"{'Tracking RMSE (rad)':<28}"
        f"{trajectory_tracking_rmse_rad(open_loop_result):>16.6f}"
        f"{trajectory_tracking_rmse_rad(impedance_result):>17.6f}"
    )
    print(
        f"{'Peak assist torque (N m)':<28}"
        f"{peak_assistive_torque_n_m(open_loop_result):>16.6f}"
        f"{peak_assistive_torque_n_m(impedance_result):>17.6f}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
