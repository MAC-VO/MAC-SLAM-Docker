#!/bin/bash
set -e

# For TensorRT & Quantization
if [[ "$CUDA_MAJOR_VERSION" = "12" ]]; then
    echo "Unable to install modelopt==0.40.0 on CUDA 12"
elif [[ "$CUDA_MAJOR_VERSION" = "13" ]]; then
    pip install "nvidia-modelopt==0.40.0"
else
    echo "!!! Error: Unsupported CUDA Major Version: $CUDA_MAJOR_VERSION"
    exit 1  
fi
