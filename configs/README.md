# Configuration

This directory contains version-controlled experiment and controller
configurations. The implemented scenarios are
`scenarios/nominal_open_loop.json` and `scenarios/nominal_tracking.json`; their
strict schema is documented in `docs/experiment_framework.md`.

The nominal values are illustrative deterministic demonstration inputs, not
identified parameters for a person or physical device.

`controllers/impedance_baseline.json` and
`controllers/computed_torque_baseline.json` contain the implemented baseline
feedback gains. Both initially use `Kp = 20 N m/rad` and `Kd = 4 N m s/rad` so
the first comparison isolates model compensation from feedback retuning. These
are engineering demonstration values, not optimized, clinically identified,
biomechanically validated, or evidence of safety. MPC, learned, and
safety-supervisor configuration remains planned.

Generated experiment results do not belong in this directory.
