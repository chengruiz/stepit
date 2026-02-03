# policy_neuro

StepIt plugin for running neural network-based policy.

### Provided Interfaces

- `stepit::neuro_policy::Actuator`: defines how to apply actions to the robot.
- `stepit::neuro_policy::Module`: provides fields according to input fields.

### Provided Factories

- `stepit::Policy`:
    - `neuro`: neural network-based locomotion policy.
- `stepit::neuro_policy::Actuator`:
    - `position`: translates actions to joint position commands.
    - `velocity`: translates actions to joint velocity commands.
    - `torque`: translates actions to joint torque commands.
    - `hybrid`: translates actions to a combination of joint commands.
- `stepit::neuro_policy::Module`:
    - `action_history`: provides history of action commands.
    - `action_filter`: applies low-pass filtering to action commands.
    - `action_reordering`: reorders action commands.
    - `actor`: infers the neural network actor to produce actions.
    - `cmd_height`: provides height command input.
    - `cmd_pitch`: provides pitch command input.
    - `cmd_roll`: provides roll command input.
    - `cmd_vel`: provides velocity command input.
    - `estimator`: infers the neural network state estimator.
    - `field_assembler`: assembles multiple fields into a single output field.
    - `field_scaling`: scales and biases input fields.
    - `heightmap`: provides heightmap input.
    - `joint_reorder`: reorders joint states.
    - `roll_pitch`: provides roll and pitch input.
    - `proprioceptor`: provides proprioceptive input.

### Joystick Key Bindings

- `LAS-X`: sets the normalized target left / right linear velocity.
- `LAS-Y`: sets the normalized target forward / backward linear velocity.
- `RAS-X`: sets the normalized target yaw rate (angular velocity).
- `RAS-Y`: sets the normalized target pitch.
- `RT`: sets the scaling factor for target velocities.
- `DPAD-LEFT` / `DPAD-RIGHT`: sets the normalized target roll.
- `DPAD-UP` / `DPAD-DOWN`: increases / decreases the target height.

## Mechanisms

### Module

Classes derived from `Module` require to declare their input dependencies (`requirements`) and output fields
(`provisions`) by registering named fields through a global FieldManager. At runtime, FieldManager assigns a unique ID
and size to each field, and maintains a registry of source factories. Then the field sources sequentially produce or
process data segments identified by FieldId.

### NeuroPolicy

`NeuroPolicy` orchestrates a set of `Module` instances into a neural‐network policy pipeline. It:

1. Loads YAML configuration and registers the main `action` field.
2. Reads user‐specified field sources and ensures an actor source is present.
3. Automatically resolves and orders sources based on declared requirements, detecting circular dependencies.
4. Calls `init()` on each source before control loop start.
5. In each `act()` step, sequentially invokes `update()` and `postUpdate()`, assembles the action vector and
   passes it to the `Agent`.
