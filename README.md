# <img src="doc/stepit.jpg" alt="StepIt Logo" width="250"/>

A flexible framework for connecting legged robots, input devices, and locomotion algorithms, especially learning-based ones.

> [!CAUTION]
> **Disclaimer: User acknowledges that all risks and consequences arising from using this code shall be solely borne by the user, the author assumes no liability for any direct or indirect damages, and proper safety measures must be implemented prior to operation.**

> **Note:** This project is under **active development**, which means the interface is unstable and breaking changes are likely to occur frequently.

## Setup

Tested on Ubuntu 20.04, 22.04 and 24.04.

```shell
sudo apt install cmake build-essential
sudo apt install libboost-dev libboost-filesystem-dev libboost-program-options-dev \
                 libeigen3-dev libfmt-dev libyaml-cpp-dev
mkdir -p stepit_ws/src && cd stepit_ws
git clone https://github.com/chengruiz/stepit.git src/stepit
```

## Build

```shell
# In the stepit_ws directory
cmake -Bbuild -Ssrc/stepit -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=install
cmake --build build -j && cmake --install build
```

You can configure the build by passing CMake variables via `-D` flags:

- `STEPIT_WHITELIST_PLUGINS` (string): semicolon-separated list of plugins to build. Default is all available plugins.
- `STEPIT_BLACKLIST_PLUGINS` (string): semicolon-separated list of plugins not to build. Default is `""`.
- `STEPIT_PLUGIN_DIRS` (string): semicolon-separated list of directories to search for plugins.

StepIt searches for plugins in directories `plugin` and the specified `STEPIT_PLUGIN_DIRS`.
All subdirectories with `CMakeLists.txt` and without `STEPIT_IGNORE` are considered plugins.
Built-in plugins are listed below.
Read the corresponding `README.md` first if you use any of the plugins.

- [`control_console`](plugin/control_console): Controlling with standard input.
- [`debugging_helper`](plugin/debugging_helper): Helper utilities for debugging.
- [`joystick_base`](plugin/joystick_base): Interface for general joystick and joystick control.
- [`joystick_udp`](plugin/joystick_udp): Controlling with retroid gamepads.
- [`joystick_usb`](plugin/joystick_usb): Controlling with USB joysticks, e.g. Xbox 360 controller.
- [`nnrt_base`](plugin/nnrt_base): Interface for neural networks.
- [`nnrt_ascendcl`](plugin/nnrt_ascendcl): Neural network inference on Ascend AI processors, e.g. OrangePI AIpro.
- [`nnrt_onnxruntime`](plugin/nnrt_onnxruntime): Neural network inference on general x86_64 processors.
- [`nnrt_rknnrt`](plugin/nnrt_rknnrt): Neural network inference on Rockchip platforms, e.g. RK3588.
- [`nnrt_tensorrt`](plugin/nnrt_tensorrt): Neural network inference on NVIDIA GPUs and Jetson platforms, e.g. Jetson
  Orin NX.
- [`policy_neuro`](plugin/policy_neuro): Neural network-based control policy.
- [`policy_neuro_ros`](plugin/policy_neuro_ros): ROS extensions for plugin `policy_neuro`.
- [`policy_neuro_ros2`](plugin/policy_neuro_ros2): ROS2 extensions for plugin `policy_neuro`.
- [`publisher_csv`](plugin/publisher_csv): Publisher for writing robot data to csv file.
- [`robot_deeprobotics_lite3`](plugin/robot_deeprobotics_lite3): Controlling the DeepRobotics Lite3 robot.
- [`robot_deeprobotics_x30`](plugin/robot_deeprobotics_x30): Controlling the DeepRobotics X30 robot (Deprecated).
- [`robot_unitree_aliengo`](plugin/robot_unitree): Controlling the Unitree Aliengo robot.
- [`robot_unitree_go1`](plugin/robot_unitree): Controlling the Unitree Go1 robot.
- [`robot_unitree_b1`](plugin/robot_unitree): Controlling the Unitree B1 robot.
- [`robot_unitree2`](plugin/robot_unitree2): Controlling Unitree Go2, B2 and G1 robots, and with Unitree joysticks.
- [`robot_unitree2_ros2`](plugin/robot_unitree2_ros2): Controlling Unitree robots with unitree_ros2.
- [`ros_base`](plugin/ros_base): ROS extensions, e.g. subscribing joysticks and publishing states.
- [`ros2_base`](plugin/ros2_base): ROS2 extensions, e.g. subscribing joysticks and publishing states.

## Run

```shell
# Run StepIt to control the robot
./install/bin/stepit
```

Command line arguments:

| Option               | Type   | Description                                             |
| -------------------- | ------ | ------------------------------------------------------- |
| `-c` / `--control`   | string | The control input type.                                 |
| `-f` / `--factory`   | string | The default factory for a specified type.               |
| `-P` / `--publisher` | string | The publisher type.                                     |
| `-p` / `--policy`    | string | The policy type and directory.                          |
| `-r` / `--robot`     | string | The controlled robot type.                              |
| `-v` / `--verbosity` | int    | Verbosity level ranging from [0, 3]. Defaults to 2.     |
| `-- [arg1 arg2 ...]` | —      | Additional arguments passed to plugins' entry function. |

Run `./install/bin/stepit --help` for more information.

## Notes

- **Environment Variable**: StepIt reads environment variables listed in [`environment.sh`](config/environment.sh). Environment variables have lower precedence than their corresponding command-line arguments.
- **Control Input**: Generally, StepIt accepts strings as control inputs, and returns a response for each one. These inputs can be provided via [the console](plugin/control_console), [joysticks](plugin/joystick_base), or topics/services in [ROS1](plugin/ros_base)/[ROS2](plugin/ros2_base). See [doc/control.md](doc/control.md) for more details.
- **State Publishing**: StepIt publishes the robot states to [CSV files](plugin/publisher_csv), or via [ROS1](plugin/ros_base) or [ROS2](plugin/ros2_base) topics.
- **Plugin Mechanism**: StepIt provides a flexible plugin architecture to enable integration of new modules without modifying existing code. See [doc/plugin.md](doc/plugin.md) for more details.
- **ROS Intergrations**: StepIt works well with ROS1 and ROS2. Make sure you read the README.md of [ROS1](plugin/ros_base) or [ROS2](plugin/ros2_base) before you start.
- **Simulation**: StepIt supports [unitree_mujoco](https://github.com/unitreerobotics/unitree_mujoco)-based simulation. See [robot_unitree2](plugin/robot_unitree2) or [stepit_sim](https://github.com/legnAray/stepit_sim) for more details.
