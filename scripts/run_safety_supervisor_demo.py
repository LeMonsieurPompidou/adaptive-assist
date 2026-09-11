"""Demonstrate deterministic safety supervision in the mathematical plant."""

from pathlib import Path

from adaptive_assist import OneDofJointModel
from adaptive_assist.controllers import (
    ImpedanceController,
    load_impedance_controller_parameters,
)
from adaptive_assist.experiments import (
    load_scenario,
    maximum_torque_modification_n_m,
    peak_assistive_torque_n_m,
    run_closed_loop_experiment,
    safety_intervention_count,
    safety_intervention_fraction,
    trajectory_tracking_rmse_rad,
)
from adaptive_assist.safety import (
    SafetyInterventionReason,
    SafetySupervisor,
    load_safety_limits,
)


def main() -> int:
    """Run the nominal impedance experiment with demonstrative limits."""
    repository_root = Path(__file__).resolve().parents[1]
    scenario = load_scenario(
        repository_root / "configs/scenarios/nominal_tracking.json"
    )
    controller = ImpedanceController(
        load_impedance_controller_parameters(
            repository_root / "configs/controllers/impedance_baseline.json"
        )
    )
    supervisor = SafetySupervisor(
        load_safety_limits(repository_root / "configs/safety/nominal_limits.json")
    )
    result = run_closed_loop_experiment(
        scenario,
        OneDofJointModel(scenario.joint_parameters),
        controller,
        supervisor,
    )
    example = next(
        sample
        for sample in result.samples
        if SafetyInterventionReason.TORQUE_LIMIT in sample.safety_intervention_reasons
    )

    print("Safety-supervisor demonstration")
    print(
        "Mathematical simulation only; not real-world, human, medical, or "
        "clinical safety validation"
    )
    print(f"Scenario: {result.scenario_name}")
    print("Controller: impedance")
    print(f"Samples: {len(result.samples)}")
    print(f"Interventions: {safety_intervention_count(result)}")
    print(f"Intervention fraction: {safety_intervention_fraction(result):.6f}")
    print(
        "Maximum torque modification: "
        f"{maximum_torque_modification_n_m(result):.6f} N m"
    )
    print(f"Tracking RMSE: {trajectory_tracking_rmse_rad(result):.6f} rad")
    print(f"Peak applied assistive torque: {peak_assistive_torque_n_m(result):.6f} N m")
    print("Example intervention:")
    print(f"requested: {example.requested_assistive_torque_n_m:.6f} N m")
    print(f"applied:   {example.applied_torques.assistive_torque_n_m:.6f} N m")
    print(
        "reason:    "
        + "|".join(reason.value for reason in example.safety_intervention_reasons)
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
