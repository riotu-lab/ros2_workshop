#!/bin/bash

pip install --user -U empy==3.3.4 pyros-genmsg
pip install setuptools==58.2.0

# Set the name of the workspace
WORKSPACE_NAME="ros2_ws"

# Set the directory where you want to create the workspace
# By default, it creates in the home directory of the current user
DIRECTORY="$HOME/$WORKSPACE_NAME"

# Check if the directory already exists
if [ -d "$DIRECTORY" ]; then
  echo "Directory $DIRECTORY already exists."
  exit 1
fi

# Create the workspace directory
echo "Creating ROS2 workspace in $DIRECTORY"
mkdir -p $DIRECTORY/src

# Navigate into the workspace directory
cd $DIRECTORY

# Initialize the workspace
echo "Initializing the workspace"
ros2 pkg create --build-type ament_cmake --node-name my_node my_package

# Build the workspace
echo "Building the workspace"
colcon build --symlink-install

echo "ROS2 workspace $WORKSPACE_NAME created and built successfully."

# Source the workspace
echo "Don't forget to source your workspace by running:"
echo "source $DIRECTORY/install/setup.bash"

