#!/bin/bash

# Change directory to ros2_ws/src
cd ~/ros2_ws/src

# Clone the ros2_mavros repository
git clone https://github.com/asmbatati/ros2_mavros.git

# Change directory to the parent directory
cd ..

# Build the ros2_mavros package
colcon build --packages-select ros2_mavros

# Append aliases to .bashrc for easy ROS2 MAVROS launch
echo "alias mavros='ros2 launch mavros px4.launch fcu_url:=\"udp://:14540@192.168.1.36:14557\"'" >> ~/.bashrc
echo "alias uav_sim='cd ~/shared_volume/PX4-Autopilot/ && make px4_sitl_default jmavsim'" >> ~/.bashrc
echo "alias x500='ros2 launch px4_offboard_control x500.launch.py'" >> ~/.bashrc
echo "alias arm='ros2 run px4_offboard_control arm'" >> ~/.bashrc
echo "alias circle='ros2 run px4_offboard_control circle'" >> ~/.bashrc
echo "alias offboard_traj='ros2 run px4_offboard_control offboard_traj'" >> ~/.bashrc

# Source .bashrc to apply changes
source ~/.bashrc

echo "Setup complete. Please close and reopen your terminal for the aliases to take effect."



