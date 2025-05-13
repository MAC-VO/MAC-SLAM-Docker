FROM nvcr.io/nvidia/pytorch:24.09-py3-igpu

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

# Set locale
RUN locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG=en_US.UTF-8

# Add ROS 2 apt repository
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | \
    gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
    http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | \
    tee /etc/apt/sources.list.d/ros2.list

# Install ROS 2 Humble
RUN apt-get update && apt-get install -y \
    ros-humble-desktop \
    python3-colcon-common-extensions \
    python3-rosdep \
    && rm -rf /var/lib/apt/lists/*

# Initialize rosdep
RUN rosdep init && rosdep update

# Source ROS 2 setup script
SHELL ["/bin/bash", "-c"]
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

# Set up the working directory
WORKDIR /home/root/workspace

# Copy and install Python dependencies
COPY ./requirements.txt /tmp/requirements.txt
RUN pip install --no-cache-dir -r /tmp/requirements.txt && rm /tmp/requirements.txt

# Configure Git
RUN git config --global --add safe.directory '*'

# Expose ports for visualization tools
EXPOSE 9876 9877 9090

CMD ["bash"]
