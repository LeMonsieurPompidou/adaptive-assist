"""Run the deterministic impedance-controller demonstration."""

from pathlib import Path

from adaptive_assist import OneDofJointModel
from adaptive_assist.controllers import (
    ImpedanceController,
    load_impedance_controller_parameters,
)
from adaptive_assist.experiments import (
    load_scenario,
    peak_assistive_torque_n_m,
    run_closed_loop_experiment,
    trajectory_tracking_rmse_rad,
)


def main() -> int:
    """Run the version-controlled tracking scenario with impedance feedback."""
    repository_root = Path(__file__).resolve().parents[1]
    scenario = load_scenario(
        repository_root / "configs/scenarios/nominal_tracking.json"
    )
    controller_parameters = load_impedance_controller_parameters(
        repository_root / "configs/controllers/impedance_baseline.json"
    )
    model = OneDofJointModel(scenario.joint_parameters)
    controller = ImpedanceController(controller_parameters)
    result = run_closed_loop_experiment(scenario, model, controller)

    print("Impedance-controller demonstration")
    print(
        "Engineering simulation only; not a safety benchmark or biomechanical "
        "or clinical validation"
    )
    print(f"Scenario: {result.scenario_name}")
    print("Controller: impedance")
    print(f"Samples: {len(result.samples)}")
    print(f"Duration: {result.metadata.duration_s:.3f} s")
    print(f"Tracking RMSE: {trajectory_tracking_rmse_rad(result):.6f} rad")
    print(f"Peak assistive torque: {peak_assistive_torque_n_m(result):.6f} N m")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
