# policy_neuro

StepIt plugin for running neural network-based policy.

### Provided Interfaces

- `stepit::neuro_policy::Actuator`: defines how to apply actions to the robot.
- `stepit::neuro_policy::Module`: provides fields according to input fields.

### Provided Factories

- `stepit::Policy`:
    - `neuro`: neural network-based locomotion policy.
- `stepit::neuro_policy::Actuator`:

  | Name       | Description                                            |
  | :--------- | :----------------------------------------------------- |
  | `position` | Translates actions to joint position commands.         |
  | `velocity` | Translates actions to joint velocity commands.         |
  | `torque`   | Translates actions to joint torque commands.           |
  | `hybrid`   | Translates actions to a combination of joint commands. |
- `stepit::neuro_policy::Module`:

  | Name                | Description                                           |
  | :------------------ | :---------------------------------------------------- |
  | `action_history`    | Provides history of action commands.                  |
  | `action_filter`     | Applies low-pass filtering to action commands.        |
  | `action_reordering` | Reorders action commands.                             |
  | `actor`             | Infers the neural network actor to produce actions.   |
  | `cmd_height`        | Provides height command input.                        |
  | `cmd_pitch`         | Provides pitch command input.                         |
  | `cmd_roll`          | Provides roll command input.                          |
  | `cmd_vel`           | Provides velocity command input.                      |
  | `estimator`         | Infers the neural network state estimator.            |
  | `field_assembler`   | Assembles multiple fields into a single output field. |
  | `field_scaling`     | Scales and biases input fields.                       |
  | `heightmap`         | Provides heightmap observations.                      |
  | `joint_reorder`     | Reorders joint states.                                |
  | `roll_pitch`        | Provides the roll and pitch observations.             |
  | `proprioceptor`     | Provides proprioceptive observations.                 |
  | `time_step`         | Provides monotonically increasing timestep index.     |

### Control Commands

- Channel: `Policy/CmdVel`

  | Action                | Argument         | Description                                  |
  | :-------------------- | :--------------- | :------------------------------------------- |
  | `SetVelocity`         | `vx`, `vy`, `wz` | Sets the target velocity commands.           |
  | `SetVelocityUnscaled` | `vx`, `vy`, `wz` | Sets the unscaled target velocity commands.  |
  | `SetTurboRatio`       | `ratio`          | Sets the turbo ratio for velocity commands.  |
  | `SelectMode`          | `mode`           | Selects the velocity control mode.           |
  | `CycleMode`           |                  | Cycles through velocity control modes.       |
  | `EnableSmoothing`     |                  | Enables velocity command smoothing.          |
  | `DisableSmoothing`    |                  | Disables velocity command smoothing.         |
  | `SetMaxAccel`         | `ax`, `ay`, `az` | Sets the maximum acceleration for smoothing. |
  | `EnableJoystick`      |                  | Enables joystick control for velocity.       |
  | `DisableJoystick`     |                  | Disables joystick control for velocity.      |

- Channel: `Policy/CmdRoll`

  | Action            | Argument | Description                          |
  | :---------------- | :------- | :----------------------------------- |
  | `SetRoll`         | `roll`   | Sets the target roll angle.          |
  | `SetRollUnscaled` | `roll`   | Sets the unscaled target roll angle. |
  | `EnableJoystick`  |          | Enables joystick control for roll.   |
  | `DisableJoystick` |          | Disables joystick control for roll.  |

- Channel: `Policy/CmdPitch`

  | Action             | Argument | Description                           |
  | :----------------- | :------- | :------------------------------------ |
  | `SetPitch`         | `pitch`  | Sets the target pitch angle.          |
  | `SetPitchUnscaled` | `pitch`  | Sets the unscaled target pitch angle. |
  | `EnableJoystick`   |          | Enables joystick control for pitch.   |
  | `DisableJoystick`  |          | Disables joystick control for pitch.  |

- Channel: `Policy/CmdHeight`

  | Action            | Argument | Description                           |
  | :---------------- | :------- | :------------------------------------ |
  | `SetHeight`       | `height` | Sets the target body height.          |
  | `IncreaseHeight`  |          | Increases the target body height.     |
  | `DecreaseHeight`  |          | Decreases the target body height.     |
  | `EnableJoystick`  |          | Enables joystick control for height.  |
  | `DisableJoystick` |          | Disables joystick control for height. |

### Joystick Key Bindings

| Key                  | Command                                    |
| :------------------- | :----------------------------------------- |
| **Start**            | `Policy/CmdVel/CycleMode`                  |
| **RT**               | `Policy/CmdVel/SetTurboRatio`              |
| **LAS-X**            | `Policy/CmdVel/SetVelocityUnscaled` (`vx`) |
| **LAS-Y**            | `Policy/CmdVel/SetVelocityUnscaled` (`vy`) |
| **RAS-X**            | `Policy/CmdVel/SetVelocityUnscaled` (`wz`) |
| **RAS-Y**            | `Policy/CmdPitch/SetPitchUnscaled`         |
| **D-Pad Up**         | `Policy/CmdHeight/IncreaseHeight`          |
| **D-Pad Down**       | `Policy/CmdHeight/DecreaseHeight`          |
| **D-Pad Left/Right** | `Policy/CmdRoll/SetRollUnscaled`           |

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
