# modular_policy_locomotion_ros

ROS extension for locomotion modules in the StepIt modular policy.

### Provided Factories

- `cmd_vel_subscriber`: reads `geometry_msgs/Twist` or `geometry_msgs/TwistStamped` into `cmd_vel` and `cmd_stall`.
- `cmd_roll_subscriber`, `cmd_pitch_subscriber`, `cmd_height_subscriber`: read `std_msgs/Float32`, `geometry_msgs/Twist`,
  or `geometry_msgs/TwistStamped` into the corresponding posture command.
- `heightmap_subscriber`: samples terrain elevation and uncertainty from `grid_map_msgs/GridMap` around the robot.

These sources support the `Policy/CmdVel`, `Policy/CmdRoll`, `Policy/CmdPitch`, `Policy/CmdHeight`, and
`Policy/Heightmap` subscriber-control commands. The generic ROS array-field subscriber remains in
`modular_policy_ros`.
