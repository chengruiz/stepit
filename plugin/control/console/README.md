# console_control

StepIt plugin providing control via standard input.
After startup, the plugin spawns a reader thread that listens on stdin.
Each non-empty line is trimmed and forwarded to the StepIt framework.

### Provided Factories

- `stepit::ControlInput`: `console`
