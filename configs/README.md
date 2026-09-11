# Configuration

This directory contains version-controlled experiment and controller
configurations. The implemented scenarios are
`scenarios/nominal_open_loop.json` and `scenarios/nominal_tracking.json`; their
strict schema is documented in `docs/experiment_framework.md`.

The nominal values are illustrative deterministic demonstration inputs, not
identified parameters for a person or physical device.

`controllers/impedance_baseline.json` contains the implemented impedance gains.
They are engineering demonstration values, not optimized, clinically
identified, biomechanically validated, or evidence of safety. Model-based,
learned, and safety-supervisor configuration remains planned.

Generated experiment results do not belong in this directory.
