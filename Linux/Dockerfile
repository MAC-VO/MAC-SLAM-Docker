ARG BASE_IMAGE="nvcr.io/nvidia/pytorch:25.06-py3"


FROM ${BASE_IMAGE}
ENV DEBIAN_FRONTEND=noninteractive


# Install additional development tools
# Dev    requirement: {tmux}
# ZedSDK requirement: {zstd}
RUN apt-get update && apt-get install -y \
    tmux \
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

# Unofficial support for Rerun > 0.23 with Numpy < 2.0
RUN  pip install --no-cache-dir --upgrade-strategy only-if-needed rerun-sdk==0.26 --no-deps

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
