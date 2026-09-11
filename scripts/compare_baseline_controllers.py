"""Compare baseline controllers and independent supervision effects."""

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
    peak_requested_assistive_torque_n_m,
    run_closed_loop_experiment,
    run_open_loop_experiment,
    safety_intervention_count,
    trajectory_tracking_rmse_rad,
)
from adaptive_assist.safety import SafetySupervisor, load_safety_limits


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


def run_supervision_comparison() -> tuple[
    ExperimentResult,
    ExperimentResult,
    ExperimentResult,
    ExperimentResult,
]:
    """Run both controllers with and without the same supervisor limits."""
    repository_root = Path(__file__).resolve().parents[1]
    scenario = load_scenario(
        repository_root / "configs/scenarios/nominal_tracking.json"
    )
    impedance = ImpedanceController(
        load_impedance_controller_parameters(
            repository_root / "configs/controllers/impedance_baseline.json"
        )
    )
    computed_torque = ComputedTorqueController(
        parameters=load_computed_torque_controller_parameters(
            repository_root / "configs/controllers/computed_torque_baseline.json"
        ),
        nominal_model=OneDofJointModel(scenario.joint_parameters),
    )
    supervisor = SafetySupervisor(
        load_safety_limits(repository_root / "configs/safety/nominal_limits.json")
    )

    impedance_unsupervised = run_closed_loop_experiment(
        scenario,
        OneDofJointModel(scenario.joint_parameters),
        impedance,
    )
    impedance_supervised = run_closed_loop_experiment(
        scenario,
        OneDofJointModel(scenario.joint_parameters),
        impedance,
        supervisor,
    )
    computed_unsupervised = run_closed_loop_experiment(
        scenario,
        OneDofJointModel(scenario.joint_parameters),
        computed_torque,
    )
    computed_supervised = run_closed_loop_experiment(
        scenario,
        OneDofJointModel(scenario.joint_parameters),
        computed_torque,
        supervisor,
    )
    return (
        impedance_unsupervised,
        impedance_supervised,
        computed_unsupervised,
        computed_supervised,
    )


def _supervision_float_row(
    label: str,
    values: tuple[float, ...],
) -> str:
    return f"{label:<29}" + "".join(f"{value:>15.6f}" for value in values)


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

    supervised_results = run_supervision_comparison()
    print()
    print("Independent safety-supervision effect")
    print(
        "The same illustrative limits are used for both controllers; this is "
        "not a ranking or real-world safety validation."
    )
    print("-" * 89)
    print(
        f"{'Metric':<29}"
        f"{'Impedance off':>15}"
        f"{'Impedance on':>15}"
        f"{'Computed off':>15}"
        f"{'Computed on':>15}"
    )
    print(
        _supervision_float_row(
            "Tracking RMSE (rad)",
            tuple(
                trajectory_tracking_rmse_rad(result) for result in supervised_results
            ),
        )
    )
    print(
        _supervision_float_row(
            "Peak requested torque (N m)",
            tuple(
                peak_requested_assistive_torque_n_m(result)
                for result in supervised_results
            ),
        )
    )
    print(
        _supervision_float_row(
            "Peak applied torque (N m)",
            tuple(peak_assistive_torque_n_m(result) for result in supervised_results),
        )
    )
    print(
        f"{'Intervention count':<29}"
        + "".join(
            f"{safety_intervention_count(result):>15d}" for result in supervised_results
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
