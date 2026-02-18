#!/bin/bash
set -euo pipefail

UBUNTU_VERSION=$(. /etc/os-release && echo "$VERSION_ID")

install_apt() {
    apt-get update
    apt-get install -y software-properties-common curl

    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
        -o /usr/share/keyrings/ros-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu jammy main" \
        > /etc/apt/sources.list.d/ros2.list

    apt-get update
    apt-get install -y ros-humble-ros-base
    rm -rf /var/lib/apt/lists/*
}

install_source() {
    apt-get update
    apt-get install -y \
        build-essential cmake git wget \
        python3-dev libpython3-dev

    pip install --no-cache-dir \
        colcon-common-extensions vcstool rosdep rosinstall-generator \
        catkin-pkg "empy<4" lark rosdistro rospkg

    # These ROS Python packages are only available as apt packages from the
    # ROS repository (not in the Ubuntu 24.04 archive). We installed them
    # via pip above, so tell rosdep to skip them.
    ROSDEP_SKIP_KEYS=(
        fastcdr
        rti-connext-dds-6.0.1
        urdfdom_headers
        python3-catkin-pkg
        python3-catkin-pkg-modules
        python3-rosdistro
        python3-rosdistro-modules
        python3-rospkg
        python3-rospkg-modules
        python3-empy
        python3-lark
    )

    rosdep init || true
    rosdep update --rosdistro humble

    mkdir -p /opt/ros2_humble_ws/src
    cd /opt/ros2_humble_ws

    rosinstall_generator ros_base --rosdistro humble --deps --tar \
        > /tmp/ros2_humble.repos
    vcs import src < /tmp/ros2_humble.repos

    rosdep install --from-paths src --ignore-src -y \
        --skip-keys "${ROSDEP_SKIP_KEYS[*]}"

    colcon build \
        --install-base /opt/ros/humble \
        --merge-install \
        --cmake-args -DCMAKE_BUILD_TYPE=Release

    cd /
    rm -rf /opt/ros2_humble_ws /tmp/ros2_humble.repos
    rm -rf /var/lib/apt/lists/*
}

case "$UBUNTU_VERSION" in
    22.04)
        echo ">>> Ubuntu 22.04 (Jammy) detected. Installing ROS 2 Humble via apt..."
        install_apt
        ;;
    24.04)
        echo ">>> Ubuntu 24.04 (Noble) detected. Building ROS 2 Humble from source..."
        install_source
        ;;
    *)
        echo "!!! Error: Unsupported Ubuntu version: $UBUNTU_VERSION"
        echo "ROS 2 Humble requires Ubuntu 22.04 (Jammy) or 24.04 (Noble)."
        exit 1
        ;;
esac

echo 'source /opt/ros/humble/setup.bash' >> /etc/bash.bashrc

# Verify installation (setup.bash uses variables that may be unset)
set +u
source /opt/ros/humble/setup.bash
set -u
python3 -c "import rclpy; rclpy.init(); rclpy.shutdown(); print('>>> rclpy OK')"
ros2 --help > /dev/null
echo ">>> ROS 2 Humble installation verified successfully."
