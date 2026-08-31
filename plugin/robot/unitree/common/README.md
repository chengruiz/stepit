# robot_unitree

StepIt plugin for controlling the Unitree robot with legacy Unitree SDK.

### Prerequisites

Install the [LCM](https://lcm-proj.github.io/lcm) library:

```shell
sudo apt install liblcm-dev
```

### Provided Factories

- `stepit::RobotApi`: `aliengo` / `go1` / `b1`

### Known issues

- The Aliengo and B1 plugins cannot be loaded together; otherwise, a warning like this will appear:
  ```
  During loading plugin libstepit_plugin_robot_aliengo.so:
    libstepit_plugin_robot_aliengo.so: undefined symbol: _ZN18UNITREE_LEGGED_SDK3UDPC1EhNS_13HighLevelTypeE.
  ```
