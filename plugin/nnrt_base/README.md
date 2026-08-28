# nnrt_base

StepIt plugin providing interface for neural network inference (runtime).

### Provided interfaces

- `stepit::Nnrt`

### Executables

- `nnrt_test`: Minimal inference test for `stepit::Nnrt`.
	- Loads a model file with optional YAML configuration.
	- Runs a few inference steps with simple constant inputs and prints outputs.
