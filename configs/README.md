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
biomechanically validated, or evidence of safety. MPC and learned-controller
configuration remains planned.

`safety/nominal_limits.json` contains the implemented deterministic simulation
supervisor limits. The values intentionally make torque clipping visible in the
nominal tracking demonstration. They are not identified human limits, device
limits, clinical thresholds, or evidence of real-world safety. Advanced safety
configuration remains planned.

`robustness/model_mismatch_sweep.json` defines the implemented deterministic
one-at-a-time plant/model mismatch evaluation. It references the nominal
tracking scenario, fixed controller gains, and one safety-limits file; varies
inertia, mass, centre-of-mass distance, damping, and stiffness at 0.8, 1.0, and
1.2 times nominal; and defines one moderate combined case. These ranges are
illustrative engineering test values, not identified human variability,
hardware tolerances, or probability distributions.

Generated experiment results do not belong in this directory.
