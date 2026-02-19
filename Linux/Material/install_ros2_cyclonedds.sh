#!/bin/bash
set -euo pipefail

UBUNTU_VERSION=$(. /etc/os-release && echo "$VERSION_ID")

install_apt() {
    apt-get update
    apt-get install -y ros-humble-rmw-cyclonedds-cpp
    rm -rf /var/lib/apt/lists/*
}

install_source() {
    set +u
    source /opt/ros/humble/setup.bash
    set -u

    rosdep update --rosdistro humble

    mkdir -p /opt/ros2_cyclonedds_ws/src
    cd /opt/ros2_cyclonedds_ws

    rosinstall_generator rmw_cyclonedds_cpp --rosdistro humble --deps --tar \
        > /tmp/ros2_cyclonedds.repos
    vcs import src < /tmp/ros2_cyclonedds.repos

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

    rosdep install --from-paths src --ignore-src -y \
        --skip-keys "${ROSDEP_SKIP_KEYS[*]}"

    colcon build \
        --install-base /opt/ros/humble \
        --merge-install \
        --cmake-args -DCMAKE_BUILD_TYPE=Release

    cd /
    rm -rf /opt/ros2_cyclonedds_ws /tmp/ros2_cyclonedds.repos
    rm -rf /var/lib/apt/lists/*
}

case "$UBUNTU_VERSION" in
    22.04)
        echo ">>> Installing CycloneDDS RMW via apt..."
        install_apt
        ;;
    24.04)
        echo ">>> Building CycloneDDS RMW from source..."
        install_source
        ;;
    *)
        echo "!!! Error: Unsupported Ubuntu version: $UBUNTU_VERSION"
        exit 1
        ;;
esac

echo ">>> CycloneDDS RMW installation completed."
