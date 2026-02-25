# nnrt_torchjit

StepIt plugin for neural network inference with TorchScript (PyTorch JIT) models.

### Prerequisites

Download the [libtorch CPU](https://pytorch.org/get-started/locally/) library, unzip it, and add the path to `CMAKE_PREFIX_PATH`. For example:

```bash
wget https://download.pytorch.org/libtorch/cpu/libtorch-shared-with-deps-2.10.0%2Bcpu.zip
unzip libtorch-shared-with-deps-2.10.0%2Bcpu.zip
mkdir -p ~/Libraries
mv libtorch-shared-with-deps-2.10.0+cpu/libtorch ~/Libraries/
echo "export CMAKE_PREFIX_PATH=\$CMAKE_PREFIX_PATH:\$HOME/Libraries/libtorch" >> ~/.bashrc  # or ~/.zshrc
```

### Provided Factories

- `stepit::NnrtApi`: `torchjit`
