#!/bin/bash
set -e

echo "Sourcing ROS environment"

# Source ROS environments
source /opt/ros/humble/setup.bash

# Set up your workspace (optional)
INSTALL_DIR="/ros2_object_detection/install"
INSTALL_SETUP_FILE=$INSTALL_DIR/setup.bash
if [[ -d $INSTALL_DIR ]]; then
    source $INSTALL_SETUP_FILE
fi

# Execute the command passed to the entrypoint
exec "$@"

# Run the following in container once started to build:
# source /opt/ros/humble/setup.bash
# rosdep update
# rosdep install --from-paths src --ignore-src -r -y
# colcon build --packages-select usb_cam camera_combiner ... (many others)
# colcon build --symlink-install --cmake-args -DCMAKE_PREFIX_PATH=/home/docker/lib/cmake --packages-select pmd_royale_ros pmd_royale_ros_driver pmd_royale_ros_examples
# source install/setup.bash