"""Run the version-controlled nominal open-loop experiment."""

import argparse
from collections.abc import Sequence
from pathlib import Path

from adaptive_assist import OneDofJointModel
from adaptive_assist.experiments import (
    load_scenario,
    peak_assistive_torque_n_m,
    run_open_loop_experiment,
    trajectory_tracking_rmse_rad,
    write_experiment_csv,
)


def main(arguments: Sequence[str] | None = None) -> int:
    """Run the nominal scenario and optionally export its samples to CSV."""
    parser = argparse.ArgumentParser(
        description="Run the deterministic nominal open-loop experiment."
    )
    parser.add_argument(
        "--csv",
        type=Path,
        help="write samples to this explicit CSV output path",
    )
    options = parser.parse_args(arguments)

    repository_root = Path(__file__).resolve().parents[1]
    scenario_path = repository_root / "configs/scenarios/nominal_open_loop.json"
    scenario = load_scenario(scenario_path)
    model = OneDofJointModel(scenario.joint_parameters)
    result = run_open_loop_experiment(scenario, model)

    print("Open-loop infrastructure demonstration (not a controller benchmark)")
    print(f"Scenario: {result.scenario_name}")
    print(f"Samples: {len(result.samples)}")
    print(f"Duration: {result.metadata.duration_s:.3f} s")
    print(f"Tracking RMSE: {trajectory_tracking_rmse_rad(result):.6f} rad")
    print(f"Peak assistive torque: {peak_assistive_torque_n_m(result):.6f} N m")

    if options.csv is not None:
        output_path = write_experiment_csv(result, options.csv)
        print(f"CSV: {output_path}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
