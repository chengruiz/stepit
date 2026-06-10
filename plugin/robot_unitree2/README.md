# robot_unitree2

StepIt plugin for controlling Unitree Go2, B2, A2, and G1 robots, and with the Unitree joysticks.

### Environment Variables

- `STEPIT_NETIF` (string, default: auto-detect): the network interface for communication. If unset, StepIt requires exactly
  one IPv4 address in the `192.168.123.x` subnet and uses its interface name automatically.
- `STEPIT_UNITREE2_DOMAIN_ID` (int, default: 0): the DDS domain ID used by unitree_sdk2.

### Provided Factories

- `stepit::RobotApi`:
  - Quadrupeds: `a2`, `b2`, `go2`, `go2w`
  - Humanoids: `g1` (29DoF), `g1_bfs` (29DoF)
- `stepit::joystick::Joystick`:
    - `unitree2`: providing joystick input with the Unitree joystick. The `LAS` button is binded to the `L1` + `L2` buttons, and the `RAS` button is binded to the `R1` + `R2` buttons.

### Executables

- `unitree2_switch_motion`: Switch or query the builtin Unitree2 locomotion state.

    ```shell
    unitree2_switch_motion help
    unitree2_switch_motion status
    unitree2_switch_motion activate <mode>
    unitree2_switch_motion deactivate
    unitree2_switch_motion disable
    unitree2_switch_motion enable
    ```

- `unitree2_switch_service`: Switch Unitree service state.

    ```shell
    unitree2_switch_service help
    unitree2_switch_service switch <service> off
    unitree2_switch_service enable <service>
    unitree2_switch_service disable <service>
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
