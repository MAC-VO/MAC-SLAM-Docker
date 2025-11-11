FROM nvcr.io/nvidia/pytorch:25.10-py3

# Set environment variables to avoid interactive prompts
ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Etc/UTC

# Install development tools and libraries
RUN apt-get update && apt-get install -y \
    build-essential \
    git vim usbutils \
    wget tmux zstd curl \
    locales tzdata \
    && rm -rf /var/lib/apt/lists/*

# Source ROS 2 setup script
SHELL ["/bin/bash", "-c"]

# Set up the working directory
WORKDIR /home/root/workspace

# Copy and install Python dependencies
COPY ./requirements.txt /tmp/requirements.txt
RUN pip install --no-cache-dir -r /tmp/requirements.txt && rm /tmp/requirements.txt

# Install uniception package, use --no-deps to avoid installing dependencies that may conflict
# with the existing environment
RUN pip install uniception --no-deps 

# Configure Git
RUN git config --global --add safe.directory '*'

# Install tmux for terminal multiplexing
RUN apt-get update && apt-get install -y tmux zstd wget &&\
    rm -rf /var/lib/apt/lists/*

# Install Zed SDK
RUN wget -O ./zed_sdk.run https://download.stereolabs.com/zedsdk/5.1/l4t38.2/jetsons && \
    chmod +x ./zed_sdk.run && \
    ./zed_sdk.run -- silent runtime_only skip_tools skip_od_module

# Expose ports for visualization tools
EXPOSE 9876 9877 9090

WORKDIR /home/root/workspace

CMD ["bash"]
