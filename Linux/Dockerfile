ARG BASE_IMAGE="nvcr.io/nvidia/pytorch:25.06-py3"


FROM ${BASE_IMAGE}
ENV DEBIAN_FRONTEND=noninteractive


# Install additional development tools
RUN apt-get update && apt-get install -y \
    tmux \
    && rm -rf /var/lib/apt/lists/*

# Setup the working directory
WORKDIR /home/root/workspace

# Freeze some key packages from the current environment
RUN pip freeze | grep -E '^(torch==|torchvision==|numpy==|tensorrt==)' \
    > /tmp/constraints.txt

# Load and install Python Requirements
COPY requirements.txt /tmp/requirements.txt
RUN  pip install --no-cache-dir --upgrade-strategy only-if-needed\
     -r /tmp/requirements.txt -c /tmp/constraints.txt \
     && rm /tmp/requirements.txt

# Configure Git
RUN git config --global --add safe.directory '*'

# Install CUDSS (For accelerated Sparse BA)
RUN wget https://developer.download.nvidia.com/compute/cudss/0.7.1/local_installers/cudss-local-repo-ubuntu2404-0.7.1_0.7.1-1_amd64.deb && \
    dpkg -i cudss-local-repo-ubuntu2404-0.7.1_0.7.1-1_amd64.deb && \
    cp /var/cudss-local-repo-ubuntu2404-0.7.1/cudss-*-keyring.gpg /usr/share/keyrings/ && \
    apt-get update && \
    apt-get -y install cudss && \
    rm ./cudss-local-repo-ubuntu2404-0.7.1_0.7.1-1_amd64.deb

# Expose rerun visualization ports
EXPOSE 9876

CMD ["bash"]
