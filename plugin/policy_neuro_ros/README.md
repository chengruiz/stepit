# policy_neuro_ros

StepIt plugin for ROS-based modules that subscribe ROS topics and feed data into the StepIt neuro policy.

### Prerequisites

```shell
sudo apt install ros-${ROS_DISTRO}-cv-bridge ros-${ROS_DISTRO}-grid-map-ros
```

### Provided Factories

`stepit::neuro_policy::Module`:
  - `cmd_height_subscriber`: subscribes to a ROS topic of one of the following types and provides command height field (`cmd_height`):
    - `std_msgs/Float32`,
    - `geometry_msgs/Twist` (`linear.z` component),
    - `geometry_msgs/TwistStamped` (`linear.z` component).
  - `cmd_pitch_subscriber`: subscribes to a ROS topic of one of the following types and provides command pitch field (`cmd_pitch`):
    - `std_msgs/Float32`,
    - `geometry_msgs/Twist` (`angular.y` component),
    - `geometry_msgs/TwistStamped` (`angular.y` component).
  - `cmd_roll_subscriber`: subscribes to a ROS topic of one of the following types and provides command roll field (`cmd_roll`):
    - `std_msgs/Float32`,
    - `geometry_msgs/Twist` (`angular.x` component),
    - `geometry_msgs/TwistStamped` (`angular.x` component).
  - `cmd_vel_subscriber`: subscribes to a ROS topic of one of the following types and provides command velocity fields (`cmd_vel`) with `linear.x`, `linear.y`, and `angular.z` components:
    - `geometry_msgs/Twist`,
    - `geometry_msgs/TwistStamped`.
  - `field_subscriber`: subscribes to configured `std_msgs/Float32MultiArray` topics and provides the named fields declared in its config map.
  - `heightmap_subscriber`: subscribes to an elevation map topic of type `grid_map_msgs/GridMap` and a pose topic of one of the following types, sample elevation and uncertainty values around the robot, and provide corresponding fields (`heightmap` / `heightmap_uncertainty`):
    - `geometry_msgs/PoseStamped`,
    - `geometry_msgs/PoseWithCovarianceStamped`,
    - `nav_msgs/Odometry`.


### Control Commands

- Channel: `Policy/CmdVel`

  | Action              | Argument | Description                          |
  | :------------------ | :------- | :----------------------------------- |
  | `EnableSubscriber`  |          | Enables subscription to ROS topics.  |
  | `DisableSubscriber` |          | Disables subscription to ROS topics. |
  | `SwitchSubscriber`  |          | Toggles subscription to ROS topics.  |

- Channel: `Policy/CmdRoll`

  | Action              | Argument | Description                          |
  | :------------------ | :------- | :----------------------------------- |
  | `EnableSubscriber`  |          | Enables subscription to ROS topics.  |
  | `DisableSubscriber` |          | Disables subscription to ROS topics. |
  | `SwitchSubscriber`  |          | Toggles subscription to ROS topics.  |

- Channel: `Policy/CmdPitch`

  | Action              | Argument | Description                          |
  | :------------------ | :------- | :----------------------------------- |
  | `EnableSubscriber`  |          | Enables subscription to ROS topics.  |
  | `DisableSubscriber` |          | Disables subscription to ROS topics. |
  | `SwitchSubscriber`  |          | Toggles subscription to ROS topics.  |

- Channel: `Policy/CmdHeight`

  | Action              | Argument | Description                          |
  | :------------------ | :------- | :----------------------------------- |
  | `EnableSubscriber`  |          | Enables subscription to ROS topics.  |
  | `DisableSubscriber` |          | Disables subscription to ROS topics. |
  | `SwitchSubscriber`  |          | Toggles subscription to ROS topics.  |

- Channel: `Policy/Heightmap`

  | Action              | Argument | Description                          |
  | :------------------ | :------- | :----------------------------------- |
  | `EnableSubscriber`  |          | Enables subscription to ROS topics.  |
  | `DisableSubscriber` |          | Disables subscription to ROS topics. |
  | `SwitchSubscriber`  |          | Toggles subscription to ROS topics.  |


### Joystick Key Bindings

| Key        | Command                             |
| :--------- | :---------------------------------- |
| **LB + A** | `Policy/CmdVel/SwitchSubscriber`    |
| **LB + A** | `Policy/CmdRoll/SwitchSubscriber`   |
| **LB + A** | `Policy/CmdPitch/SwitchSubscriber`  |
| **LB + A** | `Policy/CmdHeight/SwitchSubscriber` |
| **LB + B** | `Policy/Heightmap/SwitchSubscriber` |


### Notes

- Auto-resolution prefers `cmd_vel_subscriber` over the base `cmd_vel_source` field source because it has a higher priority. To force the non-ROS source, explicitly add `cmd_vel_source` to `modules:`. Likewise, use `cmd_roll_source`, `cmd_pitch_source`, `cmd_height_source`, or `dummy_heightmap_source` to bypass the ROS subscribers for those fields.
