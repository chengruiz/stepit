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
- [`csv_publisher`](plugin/csv_publisher): Publisher for writing robot data to csv file.
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

- `-c` / `--control` (string): The control input type.
- `-f` / `--factory` (string): The default factory for a specified type.
- `-P` / `--publisher` (string): The publisher type.
- `-p` / `--policy` (string): The policy type and directory.
- `-r` / `--robot` (string): The controlled robot type.
- `-v` / `--verbosity` (int, default: 2): Verbosity level ranging from [0, 3].
- `-- [arg1 arg2 ...]`: Additional arguments passed to plugins' entry function.

Run `./install/bin/stepit --help` for more information.
StepIt also reads environment variables, listed in [`environment.sh`](config/environment.sh).
Environment variables have lower precedence than their corresponding command-line arguments.

The state machine and transitions of StepIt are demonstrated below.

<p align="middle">
  <img src="doc/state_machine.svg" width="60%"/>
</p>

## Plugin Mechanism

StepIt provides a flexible plugin architecture for control inputs, policies, robots, and other extensions:

- **Discovery & Loading**: At runtime, `PluginManager` scans configured directories, finds shared libraries matching
  `libstepit_plugin_*.so`, and loads them dynamically.
    - By default, it searches the following directories as well as the environment variable `STEPIT_EXTRA_PLUGIN_DIRS`.
      If the environment variable `STEPIT_PLUGIN_DIRS` is specified, it overrides the default and
      `STEPIT_EXTRA_PLUGIN_DIRS`.
        - `<executable_dir>/`
        - `<executable_dir>/../lib/`
        - `<executable_dir>/../../lib/`

    - If the library exports `stepit_plugin_init`, it is invoked to allow plugins to parse command-line arguments.
    - On shutdown, `stepit_plugin_cleanup` is called if available.

- **Registration**: Plugins register the provided interfaces using the `STEPIT_REGISTER_*` macros (e.g.
  `STEPIT_REGISTER_CTRLINPUT`, `STEPIT_REGISTER_ROBOTAPI`).
    - On initialization, each registration inserts a factory into the registry, keyed by name and priority.
    - The core code retrieve a factory from the registry by name to instantiate the data types (or pick the
      highest-priority default).

This mechanism enables seamless integration of new control inputs, policies, and robot backends without modifying core
StepIt code.
