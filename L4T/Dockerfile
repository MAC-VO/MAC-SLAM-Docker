FROM nvcr.io/nvidia/pytorch:25.04-py3-igpu

# Set environment variables to avoid interactive prompts
ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Etc/UTC

# Install development tools and libraries
RUN apt-get update && apt-get install -y \
    build-essential \
    git \
    vim \
    libhdf5-dev \
    libhdf5-serial-dev \
    hdf5-tools \
    curl \
    gnupg2 \
    lsb-release \
    locales \
    tzdata \
    && rm -rf /var/lib/apt/lists/*

# Source ROS 2 setup script
SHELL ["/bin/bash", "-c"]

# Set up the working directory
WORKDIR /home/root/workspace

# Copy and install Python dependencies
COPY ./requirements.txt /tmp/requirements.txt
RUN pip install --no-cache-dir -r /tmp/requirements.txt && rm /tmp/requirements.txt

# Configure Git
RUN git config --global --add safe.directory '*'

# Install tmux for terminal multiplexing
RUN apt-get update && apt-get install -y tmux &&\
    rm -rf /var/lib/apt/lists/*

# Expose ports for visualization tools
EXPOSE 9876 9877 9090

CMD ["bash"]
