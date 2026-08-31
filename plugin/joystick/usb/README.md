# joystick_usb

StepIt plugin with USB joysticks, e.g. Xbox 360 controllers.

### Environment Variables

- `STEPIT_JS_ID` (int, default: -1): specifies the joystick ID, i.e. `/dev/input/jsX`.
  -1 means reading the first supported joystick from `/dev/input`.

### Provided Factories

- `stepit::joystick::Joystick`: `usb`
