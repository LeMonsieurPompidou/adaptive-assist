"""Run the deterministic computed-torque-controller demonstration."""

from pathlib import Path

from adaptive_assist import OneDofJointModel
from adaptive_assist.controllers import (
    ComputedTorqueController,
    load_computed_torque_controller_parameters,
)
from adaptive_assist.experiments import (
    load_scenario,
    peak_assistive_torque_n_m,
    run_closed_loop_experiment,
    trajectory_tracking_rmse_rad,
)


def main() -> int:
    """Run nominal computed-torque tracking and print its metrics."""
    repository_root = Path(__file__).resolve().parents[1]
    scenario = load_scenario(
        repository_root / "configs/scenarios/nominal_tracking.json"
    )
    controller_parameters = load_computed_torque_controller_parameters(
        repository_root / "configs/controllers/computed_torque_baseline.json"
    )
    actual_plant = OneDofJointModel(scenario.joint_parameters)
    nominal_controller_model = OneDofJointModel(scenario.joint_parameters)
    controller = ComputedTorqueController(
        parameters=controller_parameters,
        nominal_model=nominal_controller_model,
    )
    result = run_closed_loop_experiment(scenario, actual_plant, controller)

    print("Computed-torque-controller demonstration")
    print(
        "Engineering simulation only; not a safety benchmark or biomechanical "
        "or clinical validation"
    )
    print(f"Scenario: {result.scenario_name}")
    print("Controller: computed_torque")
    print(f"Samples: {len(result.samples)}")
    print(f"Duration: {result.metadata.duration_s:.3f} s")
    print(f"Tracking RMSE: {trajectory_tracking_rmse_rad(result):.6f} rad")
    print(f"Peak assistive torque: {peak_assistive_torque_n_m(result):.6f} N m")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
