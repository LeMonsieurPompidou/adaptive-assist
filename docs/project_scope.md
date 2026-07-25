# Project Scope

## Engineering problem

The project studies how to provide useful, smooth assistance at a lower-limb
joint while preserving interpretable limits on actuator behavior and joint
state. The engineering challenge is to compare controllers fairly when the
plant model is uncertain, disturbances occur, and a learned policy can improve
performance only if its authority is constrained.

The initial goal is a reproducible simulation and evaluation methodology, not a
complete wearable robot. Each controller will receive consistent observations,
references, constraints, and test scenarios so that differences can be traced
to the controller rather than the experimental setup.

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
an external simulator, reference trajectories, actuator dynamics, and parameter
ranges for comparative experiments remain future decisions.

## Target inputs and outputs

The implemented plant inputs are a joint state, human torque, assistive torque,
disturbance torque, and an explicit positive fixed time step. It outputs angular
acceleration or a new joint state for one integration step. The input state is
immutable and is not modified by stepping.

Planned controller inputs include:

- desired joint position and, where applicable, velocity and acceleration;
- measured or simulated joint position and velocity;
- estimated interaction torque or human effort;
- actuator state and configured torque and torque-rate limits; and
- optional model parameters, uncertainty descriptors, or disturbance estimates.

The future primary controller output is a commanded assistive joint torque. The
safety supervisor may constrain, filter, replace, or reject that command before
it is applied to the plant. Experiments will output time-series logs, aggregate
metrics, constraint events, configuration metadata, and reproducibility data
such as random seeds and software versions.

## Planned controllers

### Classical impedance controller

An impedance controller will provide an interpretable baseline by mapping
position and velocity tracking errors to an assistive torque. Its gains and
limits will be explicit and version controlled.

### Model-based controller

A model-based controller will use the chosen plant dynamics to provide a
dynamics-aware comparison. The exact formulation—such as computed torque or
model predictive control—remains undecided pending the plant definition and
computational requirements.

### Safe hybrid residual controller

A residual reinforcement-learning policy is planned to produce a bounded
correction to a baseline controller, rather than directly controlling the
plant. The combined command will pass through an independent safety supervisor.
Residual authority, observation design, objective, training framework, and
fallback policy remain design decisions. The learned controller will be
compared with, not substituted for, the classical baselines.

## Evaluation metrics

The initial evaluation protocol will include:

- **trajectory tracking RMSE:** root-mean-square joint position error;
- **estimated human effort:** a defined proxy based on the interaction model;
- **actuator energy:** a consistent integral based on torque and joint motion;
- **peak torque:** maximum absolute applied actuator torque;
- **torque-rate smoothness:** a measure of rapid torque changes or jerk-like
  behavior;
- **constraint violations:** counts, magnitudes, and durations by constraint;
- **robustness:** degradation under parameter variations and disturbances; and
- **repeatability:** variation across controlled random seeds where applicable.

Metric definitions, units, sampling rules, and aggregation methods must be fixed
before comparative experiments. No benchmark results currently exist.

## Safety constraints

The model deliberately implements no automatic joint limits or torque
saturation. A future safety layer will define and monitor:

- joint position and velocity bounds;
- actuator torque and torque-rate limits;
- finite observations and commands;
- bounded learned residual actions;
- deterministic fallback to a validated baseline or safe command;
- termination criteria for unsafe simulated states; and
- complete logging of supervisor interventions and violations.

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
3. **Experiment interfaces:** reference signals, scenario configuration,
   observation, logging, and metric interfaces.
4. **Baseline control:** impedance and model-based controllers with unit and
   scenario tests.
5. **Safety supervision:** command limiting, fallback behavior, termination,
   and fault-injection tests.
6. **Residual learning:** bounded residual-policy training and evaluation with
   reproducible configurations.
7. **Comparative evaluation:** nominal, uncertain, and disturbed scenario suites
   with transparent reports and ablations.
8. **Sim-to-real assessment:** decide whether evidence justifies an isolated,
   non-human bench experiment and document the transfer risks and protocol.
