# modular_policy_locomotion

StepIt plugin for the locomotion-specific inputs and observations used by modular policies.

### Provided Factories

`stepit::modular_policy::Module`:

| Name                     | Description                                                |
| :----------------------- | :--------------------------------------------------------- |
| `cmd_height_source`      | Provides a body-height command.                            |
| `cmd_pitch_source`       | Provides a body-pitch command.                             |
| `cmd_roll_source`        | Provides a body-roll command.                              |
| `cmd_vel_source`         | Provides a base-velocity command.                          |
| `dummy_heightmap_source` | Provides dummy terrain-height observations.                |
| `dummy_odometry_source`  | Provides a fallback base pose from the IMU.                |
| `proprioceptor`          | Provides IMU and joint-state observations.                 |
| `roll_pitch_source`      | Provides roll and pitch observations from the IMU.         |

The command sources accept `Policy/CmdVel`, `Policy/CmdRoll`, `Policy/CmdPitch`, and `Policy/CmdHeight` control
requests. They also install the default joystick bindings for these commands.
