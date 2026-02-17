#!/bin/bash
set -e

# Build xformers from source for NGC container (NVIDIA-optimized torch, numpy < 2.0).
# Supports: x86_64, arm64 L4T (Orin), arm64 SBSA (Thor).

# Limit build parallelism for concurrent docker compose builds (override with MAX_JOBS env if needed)
export MAX_JOBS="${MAX_JOBS:-2}"

ARCH=$(uname -m)
KERNEL=$(uname -r)

# Set TORCH_CUDA_ARCH_LIST per platform
if [[ "$ARCH" = "x86_64" ]]; then
    export TORCH_CUDA_ARCH_LIST="7.0;7.5;8.0;8.6;8.9;9.0"
elif [[ "$KERNEL" =~ tegra && "$ARCH" = "aarch64" ]]; then
    export TORCH_CUDA_ARCH_LIST="8.7"
elif [[ "$ARCH" = "aarch64" ]]; then
    export TORCH_CUDA_ARCH_LIST="9.0"
else
    echo "!!! Error: Unsupported architecture: $ARCH"
    exit 1
fi

# Pick xformers tag compatible with container's torch
TORCH_VERSION=$(python3 -c "import torch; v=torch.__version__.split('+')[0]; print(v)")
TORCH_MAJOR=$(echo "$TORCH_VERSION" | cut -d. -f1)
TORCH_MINOR=$(echo "$TORCH_VERSION" | cut -d. -f2)

if [[ "$TORCH_MAJOR" -eq 2 && "$TORCH_MINOR" -le 5 ]]; then
    XFORMERS_TAG="v0.0.28.post1"
elif [[ "$TORCH_MAJOR" -eq 2 && "$TORCH_MINOR" -ge 10 ]]; then
    XFORMERS_TAG="v0.0.34"
else
    XFORMERS_TAG="v0.0.33"
fi

echo ">>> Building xformers $XFORMERS_TAG (torch $TORCH_VERSION, arch $ARCH)"

git clone --depth 1 --branch "$XFORMERS_TAG" https://github.com/facebookresearch/xformers.git /tmp/xformers
(cd /tmp/xformers && git submodule update --init --recursive)
pip install --no-cache-dir --no-build-isolation --no-deps -c /tmp/constraints.txt /tmp/xformers
rm -rf /tmp/xformers

# Verify installation; fail build on error
python -m xformers.info || { echo "!!! Error: xformers not installed correctly"; exit 1; }
