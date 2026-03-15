# nnrt_tensorrt

StepIt plugin for neural network inference on NVIDIA GPUs and Jetson platforms, e.g. Jetson Orin NX.

### Prerequisites

NVIDIA GPU or Jetson platform with CUDA and TensorRT support (8.5.1+). The TensorRT library path should be included
in the system's library search path (e.g. via `LD_LIBRARY_PATH`).

### Provided Factories

- `stepit::NnrtApi`: `tensorrt`


### Tips: Optimize Performance on Jetson Platforms

```shell
sudo sysctl -w kernel.sched_rt_runtime_us=-1  # enable real-time scheduling
sudo jetson_clocks  # lock CPU and GPU clock frequency to maximum
```
