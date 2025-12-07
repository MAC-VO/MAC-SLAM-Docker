#!/bin/bash
set -e

ARCH=$(uname -m)
DETAIL_ARCH=$(uname -r)

if [[ "$ARCH" = "x86_64" ]]; then
    
    if [[ "$CUDA_MAJOR_VERSION" = "12" ]]; then
        # Install CUDSS for x86-64 architecture (CUDA12, CUDSS 0.6.0)
        wget https://developer.download.nvidia.com/compute/cudss/0.6.0/local_installers/cudss-local-repo-ubuntu2404-0.6.0_0.6.0-1_amd64.deb && \
            dpkg -i cudss-local-repo-ubuntu2404-0.6.0_0.6.0-1_amd64.deb && \
            cp /var/cudss-local-repo-ubuntu2404-0.6.0/cudss-*-keyring.gpg /usr/share/keyrings/ && \
            apt-get update && \
            apt-get -y install cudss && \
            rm ./cudss-local-repo-ubuntu2404-0.6.0_0.6.0-1_amd64.deb

    elif [[ "$CUDA_MAJOR_VERSION" = "13" ]]; then
        # Install CUDSS for x86-64 architecture (CUDA13, CUDSS 0.7.1)
        wget https://developer.download.nvidia.com/compute/cudss/0.7.1/local_installers/cudss-local-repo-ubuntu2404-0.7.1_0.7.1-1_amd64.deb && \
            dpkg -i cudss-local-repo-ubuntu2404-0.7.1_0.7.1-1_amd64.deb && \
            cp /var/cudss-local-repo-ubuntu2404-0.7.1/cudss-*-keyring.gpg /usr/share/keyrings/ && \
            apt-get update && \
            apt-get -y install cudss && \
            rm ./cudss-local-repo-ubuntu2404-0.7.1_0.7.1-1_amd64.deb

    else
        echo "!!! Error: Unsupported CUDA Major Version: $CUDA_MAJOR_VERSION"
        exit 1  
    fi

elif [[ "$DETAIL_ARCH" =~ "tegra" && "$ARCH" = "aarch64" ]]; then

    # Install CUDSS for Jetpack (NVIDIA Orin) architecture
    wget https://developer.download.nvidia.com/compute/cudss/0.6.0/local_installers/cudss-local-tegra-repo-ubuntu2404-0.6.0_0.6.0-1_arm64.deb && \
        dpkg -i cudss-local-tegra-repo-ubuntu2404-0.6.0_0.6.0-1_arm64.deb && \
        cp /var/cudss-local-tegra-repo-ubuntu2404-0.6.0/cudss-*-keyring.gpg /usr/share/keyrings/ && \
        apt-get update && \
        apt-get -y install cudss && \
        rm ./cudss-local-tegra-repo-ubuntu2404-0.6.0_0.6.0-1_arm64.deb

elif [[ "$ARCH" = "aarch64" ]]; then

    # Install CUDSS for ARM SBSA (NVIDIA Thor) architecture
    wget https://developer.download.nvidia.com/compute/cudss/0.7.1/local_installers/cudss-local-repo-ubuntu2404-0.7.1_0.7.1-1_arm64.deb && \
        dpkg -i cudss-local-repo-ubuntu2404-0.7.1_0.7.1-1_arm64.deb && \
        cp /var/cudss-local-repo-ubuntu2404-0.7.1/cudss-*-keyring.gpg /usr/share/keyrings/ && \
        apt-get update && \
        apt-get -y install cudss && \
        rm ./cudss-local-repo-ubuntu2404-0.7.1_0.7.1-1_arm64.deb
    
else
    echo "Error: Unsupported architecture: $ARCH"
    exit 1
fi
