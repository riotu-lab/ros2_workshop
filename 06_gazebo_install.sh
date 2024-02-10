#!/bin/bash

# Exit immediately if a command exits with a non-zero status
set -e

# Setup your computer to accept software from packages.osrfoundation.org
echo "Adding Gazebo package repository..."
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'

# Checking the file was written correctly
echo "Checking the gazebo-stable.list contents..."
cat /etc/apt/sources.list.d/gazebo-stable.list

# Setup keys
echo "Adding keys for the Gazebo repository..."
wget https://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -

# Install Gazebo
echo "Updating package lists..."
sudo apt-get update

# Ensuring the update process ends without errors
echo "Installing Gazebo 11 and development libraries..."
sudo apt-get install -y gazebo11
sudo apt-get install -y libgazebo11-dev

# Check your installation
echo "Checking Gazebo installation..."
gazebo --version

echo "Gazebo installation completed successfully."

