ARG BASE_IMAGE="nvcr.io/nvidia/pytorch:25.06-py3"
FROM ${BASE_IMAGE}

ARG  CUDA_MAJOR_VERSION="12"
ENV  DEBIAN_FRONTEND=noninteractive
ENV  CUDA_MAJOR_VERSION="${CUDA_MAJOR_VERSION}"

# Install additional development tools
# Dev    requirement: {tmux, libeigen3-dev, clangd}
# ZedSDK requirement: {zstd}
RUN apt-get update && apt-get install -y \
    tmux libeigen3-dev clangd \
    zstd \
    && rm -rf /var/lib/apt/lists/*

# Setup the working directory
WORKDIR /home/root/workspace

# Freeze some key packages from the current environment
RUN  pip freeze | grep -E '^(torch==|torchvision==|numpy==|scipy==|tensorrt==)' \
     > /tmp/constraints.txt

# Load and install Python Requirements
COPY ./Material/requirements.txt /tmp/requirements.txt
RUN  pip install --no-cache-dir --upgrade-strategy only-if-needed\
     -r /tmp/requirements.txt -c /tmp/constraints.txt \
     && rm /tmp/requirements.txt
RUN  pip install uniception==0.1.4 --no-deps

# Unofficial support for Rerun > 0.23 with Numpy < 2.0
RUN  pip install --no-cache-dir --upgrade-strategy only-if-needed rerun-sdk==0.26.0 --no-deps
RUN  curl -L https://github.com/rerun-io/rerun/releases/download/0.26.0/rerun_cpp_sdk.zip -o rerun_cpp_sdk.zip && \
    unzip rerun_cpp_sdk.zip -d /usr/local && \
    rm rerun_cpp_sdk.zip

# Configure Git
RUN  git config --global --add safe.directory '*'

# Install CUDSS (For accelerated Sparse BA)
COPY ./Material/install_cudss.sh /tmp/install_cudss.sh
RUN  bash /tmp/install_cudss.sh

# Install Zed SDK
COPY ./Material/install_zedsdk.sh /tmp/install_zedsdk.sh
RUN  bash /tmp/install_zedsdk.sh

# Expose rerun visualization ports
EXPOSE 9876

CMD ["bash"]
