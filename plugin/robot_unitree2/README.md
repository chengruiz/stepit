# robot_unitree2

StepIt plugin for controlling the Unitree Go2, B2 and G1 robots, and with the Unitree joysticks.

### Environment Variables

- `STEPIT_NETIF` (string, default: eth0): the network interface for communication.

### Provided Factories

- `stepit::RobotApi`: `b2`, `go2`, `g1_15dof`, `g1_23dof`, `g1_29dof`
- `stepit::joystick::Joystick`:
    - `unitree2`: providing joystick input with the Unitree joystick. The `LAS` button is binded to the `L1` + `L2` buttons, and the `RAS` button is binded to the `R1` + `R2` buttons.

### Notes

- Add unitree_sdk2 thirdparty library path to the front of `LD_LIBRARY_PATH` if stepit is also built with ROS2.

```bash
export LD_LIBRARY_PATH=<stepit_dir>/extern/robot_sdk/unitree_sdk2/thirdparty/lib/$(uname -m):$LD_LIBRARY_PATH
```

- Set environment variable `STEPIT_NETIF` to `lo` to use [unitree_mujoco](https://github.com/unitreerobotics/unitree_mujoco) for sim-to-sim transfer.

```bash
export STEPIT_NETIF=lo
```
