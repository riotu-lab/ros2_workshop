#!/bin/bash

# Exit immediately if a command exits with a non-zero status.
set -e

# Setup the ROS 2 Environment Variables
echo "Setting up ROS 2 Environment Variables..."
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
echo "ROS 2 environment variables set."

# Install Gazebo Simulator
echo "Installing Gazebo Simulator..."
#sudo apt update && sudo apt upgrade -y
#sudo add-apt-repository universe
#sudo apt update

#sudo apt install -y gazebo11
#sudo apt remove gz-tools2
#sudo apt install -y ros-humble-gazebo-ros-pkgs
echo "Gazebo installed."

# Install ROS 2 Dependent Packages
echo "Installing ROS 2 Dependent Packages..."

# Cartographer
echo "Installing Cartographer..."
sudo apt install -y ros-humble-cartographer 
sudo apt install -y ros-humble-cartographer-ros

# Navigation Stack
echo "Installing Navigation Stack for ROS 2..."
sudo apt install -y ros-humble-navigation2 
sudo apt install -y ros-humble-nav2-bringup
echo "ROS 2 Dependent Packages installed."

# Create a ROS2 Workspace
#echo "Creating ROS 2 Workspace..."
#mkdir -p ~/turtlebot3_ws/src
#echo "ROS 2 Workspace created at ~/turtlebot3_ws."

# Install Turtlebot3 Packages
echo "Installing Turtlebot3 Packages..."
cd ~/ros2_ws/src
git clone https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git -b humble-devel
git clone https://github.com/ROBOTIS-GIT/turtlebot3.git -b humble-devel
git clone https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git -b humble-devel
git clone https://github.com/ROBOTIS-GIT/DynamixelSDK.git -b humble-devel
echo "Turtlebot3 packages cloned."

# Build the Packages
echo "Building the Turtlebot3 packages..."
cd ~/ros2_ws
colcon build
source ~/ros2_ws/install/setup.bash
echo "Turtlebot3 packages built."

# Set Turtlebot3 Model
echo "Setting TURTLEBOT3_MODEL to burger..."
echo 'export TURTLEBOT3_MODEL=burger' >> ~/.bashrc
source ~/.bashrc
echo "TURTLEBOT3_MODEL set to burger."

echo "Installation complete. You can now run Turtlebot3 simulations."

# The script ends here. The user will manually source and run simulations as per the guide.

