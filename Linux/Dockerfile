ARG BASE_IMAGE="nvcr.io/nvidia/pytorch:25.06-py3"
FROM ${BASE_IMAGE}

ARG  CUDA_MAJOR_VERSION="12"
ENV  DEBIAN_FRONTEND=noninteractive
ENV  CUDA_MAJOR_VERSION="${CUDA_MAJOR_VERSION}"

# Install additional development tools
# Dev    requirement: {tmux, libeigen3-dev, libopencv-dev, clangd}
# ZedSDK requirement: {zstd}
# NOTE:
# Installing Ubuntu OpenCV packages can pull system UCX variants that conflict with
# the HPCX UCX/UCC stack bundled in the NGC base image. Keep package install minimal,
# and force HPCX search order to avoid torch import failures.
RUN apt-get update && apt-get install -y --no-install-recommends \
    tmux libeigen3-dev libopencv-dev clangd pkg-config libboost-dev libboost-serialization-dev libboost-all-dev \
    zstd \
    && rm -rf /var/lib/apt/lists/*

# Prefer bundled HPCX UCX/UCC runtime over distro UCX to keep torch import stable.
ENV LD_LIBRARY_PATH="/opt/hpcx/ucx/lib:/opt/hpcx/ucc/lib:${LD_LIBRARY_PATH}"

# Setup the working directory
WORKDIR /home/root/workspace

# Freeze some key packages from the current environment
RUN  pip freeze | grep -E '^(torch==|torchvision==|numpy==|scipy==|tensorrt==|protobuf==)' \
     > /tmp/constraints.txt

# Load and install Python Requirements
COPY ./Material/requirements.txt /tmp/requirements.txt
RUN  pip install --no-cache-dir --upgrade-strategy only-if-needed\
     -r /tmp/requirements.txt -c /tmp/constraints.txt \
     && rm /tmp/requirements.txt
# For UFM and MapAnything
RUN  pip install uniception==0.1.4 --no-deps

# Unofficial support for Rerun > 0.23 with Numpy < 2.0
RUN  pip install --no-cache-dir --upgrade-strategy only-if-needed rerun-sdk==0.32.0 --no-deps
RUN  curl -L https://github.com/rerun-io/rerun/releases/download/0.32.0/rerun_cpp_sdk.zip -o rerun_cpp_sdk.zip && \
    unzip rerun_cpp_sdk.zip -d /usr/local && \
    rm rerun_cpp_sdk.zip

# Configure Git
RUN  git config --global --add safe.directory '*'

# if CUDA_MAJOR_VERSION is 13, install cv2
COPY ./Material/install_cv2.sh /tmp/install_cv2.sh
RUN  bash /tmp/install_cv2.sh

# Install ModelOpt for Quantization
COPY ./Material/install_modelopt.sh /tmp/install_modelopt.sh
RUN  bash /tmp/install_modelopt.sh

# Install CUDSS (For accelerated Sparse BA)
COPY ./Material/install_cudss.sh /tmp/install_cudss.sh
RUN  bash /tmp/install_cudss.sh

# Install Zed SDK
COPY ./Material/install_zedsdk.sh /tmp/install_zedsdk.sh
RUN  bash /tmp/install_zedsdk.sh

# Install GT-SAM
COPY ./Material/install_gtsam.sh /tmp/install_gtsam.sh
RUN  bash /tmp/install_gtsam.sh

# Non-root runtime: sudo NOPASSWD (skip pam_unix shadow lookup since we don't
# bind-mount /etc/shadow), system-wide git safe.directory, and restore world-
# read on /usr/local (NGC base ships some site-packages as mode 0600).
# Kept last so it doesn't bust the cuDSS/GTSAM cache above.
RUN apt-get update && apt-get install -y --no-install-recommends sudo \
    && rm -rf /var/lib/apt/lists/* \
    && echo "ALL ALL=(ALL) NOPASSWD: ALL" > /etc/sudoers.d/99-macslam-nopasswd \
    && chmod 0440 /etc/sudoers.d/99-macslam-nopasswd \
    && sed -i 's/^@include common-account/account required pam_permit.so/' /etc/pam.d/sudo \
    && git config --system --add safe.directory '*' \
    && chmod -R go+rX /usr/local

# Expose rerun visualization ports
EXPOSE 9876

CMD ["bash"]
