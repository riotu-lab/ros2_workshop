#!/bin/bash

# Exit immediately if a command exits with a non-zero status.
set -e

# Define workspace name
WORKSPACE_NAME=ros2_ws
pip install setuptools==58.2.0

# PX4 Autopilot Installation
echo "Cloning PX4 Autopilot..."
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
echo "Installing PX4 dependencies..."
bash ./PX4-Autopilot/Tools/setup/ubuntu.sh 
echo "make sure to remove files not compatible with unbuntu.sh with ARM64 architecture."
echo "PX4 Autopilot installation completed. Please restart your computer before continuing."

# Step 1: Clean the repository and submodules
echo "Cleaning repository and submodules..."
make submodulesclean
make clean
make distclean

# Step 2: Checkout to specific version
echo "Checking out to version v1.14.0..."
git checkout v1.14.0

# Step 3: Clean the repository and submodules again
echo "Performing a second round of cleaning after checkout..."
make submodulesclean
make clean
make distclean

echo "Repository is now at version v1.14.0 and cleaned."

cd ~/PX4-Autopilot
git submodule update --init --recursive
make px4_sitl jmavsim



# Python dependencies
echo "Installing Python dependencies..."
pip3 install --user -U empy==3.3.4 pyros-genmsg setuptools kconfiglib jsonschema jinja2

# Micro XRCE-DDS Agent Installation
echo "Cloning and building Micro XRCE-DDS Agent..."
git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
cd Micro-XRCE-DDS-Agent
mkdir build && cd build
cmake ..
make
sudo make install
sudo ldconfig /usr/local/lib/
cd ../..

# Workspace setup
echo "Setting up the ROS2 workspace..."
cd ~/$WORKSPACE_NAME

#pip3 uninstall em
#pip3 install empy
pip install --user -U empy==3.3.4 pyros-genmsg setuptools

# Clone the entire workshop repository
echo "Cloning the ROS2 workshop repository..."
git clone https://github.com/riotu-lab/ros2_workshop.git src/

# Build the workspace
echo "Building the workspace..."
source /opt/ros/humble/setup.bash
colcon build

# Source the workspace automatically by adding it to .bashrc
echo "source ~/$WORKSPACE_NAME/install/setup.bash" >> ~/.bashrc

echo "Workspace setup and build completed."
echo "Please source your .bashrc or restart the terminal."
echo "source ~/.bashrc"

# Reminder for manual steps if any
echo "Please remember to restart your computer before continuing with the ROS2 workspace usage."

