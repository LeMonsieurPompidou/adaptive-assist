# Project Scope

## Engineering problem

The project studies how to provide useful, smooth assistance at a lower-limb
joint while preserving interpretable limits on actuator behavior and joint
state. The engineering challenge is to compare controllers fairly when the
plant model is uncertain, disturbances occur, and a learned policy can improve
performance only if its authority is constrained.

The implemented foundation provides reproducible open- and closed-loop
execution and evaluation methodology, not a complete wearable robot. The
impedance and computed-torque baselines reuse consistent references, records,
metrics, and test scenarios so experimental differences can be traced to
assistive-torque generation rather than the surrounding infrastructure.

## One-degree-of-freedom abstraction

The implemented mathematical plant represents one actuated revolute lower-limb
joint coupled to a simplified distal mass. Its state contains joint angle and
angular velocity. Configurable constant parameters represent inertia, mass,
centre-of-mass distance, gravity, viscous damping, passive stiffness, and a
passive rest angle. Human, assistive, and disturbance torques are explicit
inputs.

The scalar model uses the standard library and semi-implicit Euler integration.
Its governing equation, sign convention, assumptions, validation rules, and SI
units are defined in [the model documentation](one_dof_model.md). Selection of
an external simulator, a broader reference suite for controller comparisons,
actuator dynamics, and parameter ranges remain future decisions.

## Target inputs and outputs

The implemented plant inputs are a joint state, human torque, assistive torque,
disturbance torque, and an explicit positive fixed time step. It outputs angular
acceleration or a new joint state for one integration step. The input state is
immutable and is not modified by stepping.

The implemented experiment layer loads a versioned scenario containing these
values, evaluates a constant or sinusoidal `JointReference`, executes predefined
open-loop torques, and produces immutable time-series samples and deterministic
metadata. It can explicitly export samples to CSV and currently calculates
tracking RMSE and peak assistive torque. These are infrastructure metrics, not
controller benchmark results.

The implemented impedance-controller inputs are desired and actual joint angle
and angular velocity. The computed-torque controller additionally uses desired
angular acceleration and a separate nominal `OneDofJointModel`. Future
controller inputs may additionally include:

- desired joint position and, where applicable, velocity and acceleration;
- measured or simulated joint position and velocity;
- estimated interaction torque or human effort;
- actuator state and configured torque and torque-rate limits; and
- optional model parameters, uncertainty descriptors, or disturbance estimates.

The implemented controller output is requested assistive joint torque. The
optional mathematical safety supervisor can replace or clip that request before
the runner passes applied assistive torque to the plant. Direct pass-through
remains available and is explicit in experiment metadata. Current experiments
output in-memory time-series samples, deterministic metadata, requested/applied
commands, intervention reasons, optional CSV, and controller-independent
metrics. Richer provenance and random-seed reporting if randomness is later
introduced remain planned.

## Controller status

### Classical impedance controller — implemented

The impedance controller provides an interpretable baseline by mapping position
and velocity tracking errors to a requested assistive torque. Its non-negative
proportional and derivative gains are explicit and version controlled. It has
no saturation, actuator limit, or safety function.

### Computed-torque model-based controller — implemented

The computed-torque controller combines the same proportional and derivative
tracking feedback with desired-acceleration inertial feedforward and
current-state gravity and passive-torque compensation. It reuses public plant
methods through a separate nominal model object. Human and disturbance torques
remain external inputs and are not canceled. MPC remains planned and is not
part of this baseline.

### Safe hybrid residual controller

A residual reinforcement-learning policy is planned to produce a bounded
correction to a baseline controller, rather than directly controlling the
plant. The combined command will pass through an independent safety supervisor.
Residual authority, observation design, objective, training framework, and
fallback policy remain design decisions. The learned controller will be
compared with, not substituted for, the classical baselines.

## Evaluation metrics

The initial evaluation protocol will include:

- **trajectory tracking RMSE:** root-mean-square joint position error
  (implemented);
- **estimated human effort:** a defined proxy based on the interaction model;
- **actuator energy:** a consistent integral based on torque and joint motion;
- **peak torque:** maximum absolute applied assistive torque (implemented);
- **torque-rate smoothness:** a measure of rapid torque changes or jerk-like
  behavior;
- **simulation supervisor interventions:** count, sample fraction, and maximum
  requested-to-applied torque modification (implemented);
- **constraint violations:** richer counts, magnitudes, and durations by
  constraint;
- **robustness:** degradation under parameter variations and disturbances; and
- **repeatability:** variation across controlled random seeds where applicable.

The implemented metrics have fixed code-level definitions and are used by the
current engineering comparisons. Peak assistive torque means applied torque;
peak requested torque is available separately. Definitions, units, sampling
rules, and aggregation methods for remaining metrics must be fixed before formal
benchmarks. Current demonstration outputs are not benchmark results or safety
evidence.

## Safety constraints

The plant and controllers deliberately implement no joint limits or torque
saturation. The independent mathematical safety supervisor currently provides:

- fallback for non-finite requested commands;
- current joint-angle and absolute-velocity checks;
- symmetric assistive-torque clipping; and
- deterministic per-sample intervention reasons.

Illustrative supervisor values are not human, device, medical, or clinical
limits. Future work may define and evaluate:

- predictive enforcement or termination semantics for joint position and
  velocity bounds;
- torque-rate and actuator-state limits;
- broader observation validation;
- bounded learned residual actions;
- deterministic fallback to a validated baseline or safe command;
- predictive constraints and termination criteria; and
- richer intervention and violation accounting.

Safety logic should be independent of the learned policy and exercised by unit,
integration, and fault-injection tests. These engineering constraints do not
constitute medical or clinical validation.

## Assumptions

The initial scope assumes:

- a single revolute joint captures enough behavior to compare control concepts;
- state measurements or estimates are available at a fixed control rate;
- reference trajectories are bounded and feasible under nominal conditions;
- interaction with a person is represented only by a mathematical model or
  non-human physical load;
- simulation parameters and uncertainty ranges can be made explicit; and
- controller comparisons use identical scenarios and evaluation definitions.

These assumptions will be revisited as evidence is collected.

## Explicit non-goals

The project does not currently pursue:

- medical validation or regulatory approval;
- clinical claims of safety, efficacy, rehabilitation, or health benefit;
- control of a device worn by a person;
- direct human testing;
- immediate development of a complete exoskeleton;
- multi-joint locomotion control;
- autonomous diagnosis or treatment; or
- claims that simulation alone establishes real-world safety.

## Phased milestones

1. **Project foundation — complete:** packaging, documentation, automated
   quality checks, and continuous integration.
2. **Deterministic plant model — complete:** governing scalar equation,
   parameters, SI units, torque inputs, validation, fixed-step integration, and
   behavior tests.
3. **Experiment interfaces — complete:** constant and sinusoidal reference
   signals, strict JSON scenarios, deterministic open-loop execution, immutable
   records, CSV logging, and initial metrics.
4. **Baseline control — complete for the current scope:** deterministic
   impedance and computed-torque control, generic closed-loop execution, and
   equivalent-condition comparison are implemented. MPC remains planned.
5. **Simulation safety supervision — complete for the current scope:**
   finite-command fallback, current-state limit checks, torque clipping,
   intervention records, metrics, and tests. Predictive and real-world safety
   work remains outside the implemented scope.
6. **Residual learning:** bounded residual-policy training and evaluation with
   reproducible configurations.
7. **Comparative evaluation:** nominal, uncertain, and disturbed scenario suites
   with transparent reports and ablations.
8. **Sim-to-real assessment:** decide whether evidence justifies an isolated,
   non-human bench experiment and document the transfer risks and protocol.
