# robot_unitree2

StepIt plugin for controlling the Unitree Go2, B2 and G1 robots, and with the Unitree joysticks.

### Environment Variables

- `STEPIT_NETIF` (string, default: eth0): the network interface for communication.
- `STEPIT_UNITREE2_DOMAIN_ID` (int, default: 0): the DDS domain ID used by unitree_sdk2.

### Provided Factories

- `stepit::RobotApi`:
  - Quadrupeds: `b2`, `go2`
  - Humanoids: `g1`, `g1_15dof`, `g1_23dof`, `g1_bfs`
- `stepit::joystick::Joystick`:
    - `unitree2`: providing joystick input with the Unitree joystick. The `LAS` button is binded to the `L1` + `L2` buttons, and the `RAS` button is binded to the `R1` + `R2` buttons.

### Executables

- `unitree2_switch_mode`: Switch or query the builtin Unitree2 locomotion state.

    ```shell
    unitree2_switch_mode help
    unitree2_switch_mode status
    unitree2_switch_mode activate <mode>
    unitree2_switch_mode deactivate
    unitree2_switch_mode disable
    unitree2_switch_mode enable
    ```

### Notes

- Add unitree_sdk2 thirdparty library path to the front of `LD_LIBRARY_PATH` if stepit is also built with ROS2.

    ```shell
    export LD_LIBRARY_PATH=<stepit_dir>/extern/robot_sdk/unitree_sdk2/thirdparty/lib/$(uname -m):$LD_LIBRARY_PATH
    ```

- Set environment variable `STEPIT_NETIF` to `lo` to use [unitree_mujoco](https://github.com/unitreerobotics/unitree_mujoco) for sim-to-sim transfer.

    ```shell
    export STEPIT_NETIF=lo
    ```
