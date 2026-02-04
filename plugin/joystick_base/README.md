# joystick_base

StepIt plugin providing joystick interface and control.

### Provided Interfaces

- `stepit::joystick::Joystick`

### Provided Factories

- `stepit::ControlInput`: `joystick`

### Joystick Key Bindings

| Key                   | Command                  |
| :-------------------- | :----------------------- |
| `LAS`[^1] + `RAS`[^2] | `Agent/Freeze`           |
| `LT`[^3] + `X`        | `Agent/Unfreeze`         |
| `LT` + `A`            | `Agent/StandUpOrLieDown` |
| `LT` + `B`            | `Agent/PolicyOnOrOff`    |
| `LT` + `Y`            | `Agent/CyclePolicy`      |

[^1]: `LAS` refers to pressing the left analog stick.

[^2]: `RAS` refers to pressing the right analog stick.

[^3]: `LT` refers to the left trigger (or `L2`).
