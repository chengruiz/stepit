# pyutils

StepIt plugin containing Python-based utilities for C++ modules.

### Prerequisites

Install the [pybind11](https://pybind11.readthedocs.io/en/stable/) library:

```shell
sudo apt install python3-pybind11
```

### Executables

- `npz_info`: prints key/shape/dtype summary for arrays in an `.npz` file.

	```shell
	npz_info <npz_file>
	```
