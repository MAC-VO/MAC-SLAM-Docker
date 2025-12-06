#!/bin/bash
set -e

# 1. Detect the version
# We rely on the $CUDA_VERSION environment variable provided by the base image.
# If it's not set, default to a fallback or exit error.
CURRENT_VER=${CUDA_VERSION:-"unknown"}

echo "Running installation script..."
echo "Detected full CUDA Version: $CURRENT_VER"

# Extract the major version (e.g., "12" from "12.2.0")
MAJOR_VER=$(echo "$CURRENT_VER" | cut -d. -f1)

# 2. Conditional Logic
if [ "$MAJOR_VER" = "12" ]; then
    echo ">>> Branch: CUDA 12 Detected."
    echo ">>> Installing ZedSDK for CUDA 12..."
    
    # commands for CUDA 12
    wget -O /tmp/zed_sdk.run https://download.stereolabs.com/zedsdk/5.1/cu12/ubuntu24 && \
        chmod +x /tmp/zed_sdk.run && \
        /tmp/zed_sdk.run -- silent runtime_only skip_tools skip_od_module

elif [ "$MAJOR_VER" = "13" ]; then
    echo ">>> Branch: CUDA 13 Detected."
    echo ">>> Installing ZedSDK for CUDA 13..."

    # commands for CUDA 13
    wget -O /tmp/zed_sdk.run https://download.stereolabs.com/zedsdk/5.1/cu13/ubuntu24 && \
        chmod +x /tmp/zed_sdk.run && \
        /tmp/zed_sdk.run -- silent runtime_only skip_tools skip_od_module

else
    echo "!!! Error: Unsupported CUDA Major Version: $MAJOR_VER"
    exit 1
fi

echo ">>> ZedSDK Installation completed successfully."
