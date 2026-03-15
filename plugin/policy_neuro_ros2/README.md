# policy_neuro_ros2

StepIt plugin for ROS2-based modules that subscribe to ROS2 topics and feed data into the StepIt neuro policy.

### Prerequisites

```shell
sudo apt install ros-${ROS_DISTRO}-cv-bridge ros-${ROS_DISTRO}-grid-map-ros
```

### Provided Factories

`stepit::neuro_policy::Module`:

- `cmd_height_subscriber`: subscribes to a ROS2 topic of one of the following types and provides command height field (`cmd_height`):
    - `std_msgs/msg/Float32`,
    - `geometry_msgs/msg/Twist` (`linear.z` component),
    - `geometry_msgs/msg/TwistStamped` (`linear.z` component).
- `cmd_pitch_subscriber`: subscribes to a ROS2 topic of one of the following types and provides command pitch field (`cmd_pitch`):
    - `std_msgs/msg/Float32`,
    - `geometry_msgs/msg/Twist` (`angular.y` component),
    - `geometry_msgs/msg/TwistStamped` (`angular.y` component).
- `cmd_roll_subscriber`: subscribes to a ROS2 topic of one of the following types and provides command roll field (`cmd_roll`):
    - `std_msgs/msg/Float32`,
    - `geometry_msgs/msg/Twist` (`angular.x` component),
    - `geometry_msgs/msg/TwistStamped` (`angular.x` component).
- `cmd_vel_subscriber`: subscribes to a ROS2 topic of one of the following types and provides command velocity fields (`cmd_vel`) with `linear.x`, `linear.y`, and `angular.z` components:
    - `geometry_msgs/msg/Twist`,
    - `geometry_msgs/msg/TwistStamped`.
- `field_subscriber`: subscribes to configured `std_msgs/msg/Float32MultiArray` topics and provides the named fields declared in its config map.
- `heightmap_subscriber`: subscribes to an elevation map topic of type `grid_map_msgs/msg/GridMap` and a pose topic of one of the following types, sample elevation and uncertainty values around the robot, and provide corresponding fields (`heightmap` / `heightmap_uncertainty`):
    - `geometry_msgs/msg/PoseStamped`,
    - `geometry_msgs/msg/PoseWithCovarianceStamped`,
    - `nav_msgs/msg/Odometry`.


### Control Commands

- Channel: `Policy/CmdVel`

  | Action              | Argument | Description                           |
  | :------------------ | :------- | :------------------------------------ |
  | `EnableSubscriber`  |          | Enables subscription to ROS2 topics.  |
  | `DisableSubscriber` |          | Disables subscription to ROS2 topics. |
  | `SwitchSubscriber`  |          | Toggles subscription to ROS2 topics.  |

- Channel: `Policy/CmdRoll`

  | Action              | Argument | Description                           |
  | :------------------ | :------- | :------------------------------------ |
  | `EnableSubscriber`  |          | Enables subscription to ROS2 topics.  |
  | `DisableSubscriber` |          | Disables subscription to ROS2 topics. |
  | `SwitchSubscriber`  |          | Toggles subscription to ROS2 topics.  |

- Channel: `Policy/CmdPitch`

  | Action              | Argument | Description                           |
  | :------------------ | :------- | :------------------------------------ |
  | `EnableSubscriber`  |          | Enables subscription to ROS2 topics.  |
  | `DisableSubscriber` |          | Disables subscription to ROS2 topics. |
  | `SwitchSubscriber`  |          | Toggles subscription to ROS2 topics.  |

- Channel: `Policy/CmdHeight`

  | Action              | Argument | Description                           |
  | :------------------ | :------- | :------------------------------------ |
  | `EnableSubscriber`  |          | Enables subscription to ROS2 topics.  |
  | `DisableSubscriber` |          | Disables subscription to ROS2 topics. |
  | `SwitchSubscriber`  |          | Toggles subscription to ROS2 topics.  |

- Channel: `Policy/Heightmap`

  | Action              | Argument | Description                           |
  | :------------------ | :------- | :------------------------------------ |
  | `EnableSubscriber`  |          | Enables subscription to ROS2 topics.  |
  | `DisableSubscriber` |          | Disables subscription to ROS2 topics. |
  | `SwitchSubscriber`  |          | Toggles subscription to ROS2 topics.  |


### Joystick Key Bindings

| Key        | Command                             |
| :--------- | :---------------------------------- |
| **LB + A** | `Policy/CmdVel/SwitchSubscriber`    |
| **LB + A** | `Policy/CmdRoll/SwitchSubscriber`   |
| **LB + A** | `Policy/CmdPitch/SwitchSubscriber`  |
| **LB + A** | `Policy/CmdHeight/SwitchSubscriber` |
| **LB + B** | `Policy/Heightmap/SwitchSubscriber` |


### Notes

- Auto-resolution prefers `cmd_vel_subscriber` over the base `cmd_vel_source` field source because it has a higher priority. To force the non-ROS2 source, explicitly add `cmd_vel_source` to `modules:`. Likewise, use `cmd_roll_source`, `cmd_pitch_source`, `cmd_height_source`, or `dummy_heightmap_source` to bypass the ROS2 subscribers for those fields.
