# ROS2 Workshop Packages

Welcome to the **ROS2 workshop** Git repository.

[![Course](https://img.shields.io/badge/Udemy-Course-blue)](https://www.udemy.com/course/ros2-how-to/)
[![Discount](https://img.shields.io/badge/Discount-Coupons-green)](https://www.riotu-lab.org/udemy.php)

## Overview & Resources

This repository contains the packages and materials designed to help you learn essential ROS2 concepts and gain hands-on experience in both C++ and Python languages.

This tutorial explains at a basic level how to use ROS2 and PX4 in order to control a simulated UAV's velocity with keyboard controls. The goal is to create a simple example that a complete beginner can follow and understand, even with no ROS2 or PX4 experience.

This repo is a derivative of Jaeyoung Lim's Offboard example
https://github.com/Jaeyoung-Lim/px4-offboard

I've taken his example and added some functionality. 

### Prerequisites
* ROS2 Humble
* PX4 Autopilot
* Micro XRCE-DDS Agent
* px4_msgs
* Ubuntu 22.04
* Python 3.10

## Setup Steps

### Install PX4 Autopilot
To [Install PX4](https://docs.px4.io/main/en/dev_setup/dev_env_linux_ubuntu.html#simulation-and-nuttx-pixhawk-targets) run this code 
```
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
```

Run this script in a bash shell to install everything

```
bash ./PX4-Autopilot/Tools/setup/ubuntu.sh
```

You will now need to restart your computer before continuing.


### Install ROS2 Humble
To install ROS2 Humble follow the steps [here](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)

### Install Dependencies

Install Python dependencies as mentioned in the [PX4 Docs](https://docs.px4.io/main/en/ros/ros2_comm.html#install-ros-2) with this code

```
pip3 install --user -U empy pyros-genmsg setuptools
```

I also found that without these packages installed Gazebo has issues loading

```
pip3 install kconfiglib
pip install --user jsonschema
pip install --user jinja2
```

### Build Micro DDS
As mentioned in the [PX4 Docs](https://docs.px4.io/main/en/ros/ros2_comm.html#setup-micro-xrce-dds-agent-client) run this code in order to build MicroDDS on your machine

```
git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
cd Micro-XRCE-DDS-Agent
mkdir build
cd build
cmake ..
make
sudo make install
sudo ldconfig /usr/local/lib/
```

### Setup Workspace
This git repo is intended to be a ROS2 package that is cloned into a ROS2 workspace.

We're going to create a workspace in our home directory, and then clone in this repo and also the px4_msgs repo. 

For more information on creating workspaces, see [here](https://docs.ros.org/en/humble/Tutorials/Workspace/Creating-A-Workspace.html)

Run this code to create a workspace in your home directory

```
mkdir -p ~/ros2_px4_offboard_example_ws/src
cd ~/ros2_px4_offboard_example_ws/src
```

*ros2_px4_offboard_example_ws* is just a name I chose for the workspace. You can name it whatever you want. But after we run *colcon build* you might have issues changing your workspace name so choose wisely.

We are now in the src directory of our workspace. This is where ROS2 packages go, and is where we will clone in our two repos.

### Clone in Packages
We first will need the px4_msgs package. Our ROS2 nodes will rely on the message definitions in this package in order to communicate with PX4. Read [here](https://docs.px4.io/main/en/ros/ros2_comm.html#overview:~:text=ROS%202%20applications,different%20PX4%20releases) for more information.

Be sure you're in the src directory of your workspace and then run this code to clone in the px4_msgs repo

```
git clone https://github.com/PX4/px4_msgs.git
```

Once again be sure you are still in the src directory of your workspace. Run this code to clone in our example package

```
git clone https://github.com/ARK-Electronics/ROS2_PX4_Offboard_Example.git
```

Run this code to clone the repo

### Building the Workspace
The two packages in this workspace are px4_msgs and px4_offboard. px4_offboard is a ROS2 package that contains the code for the offboard control node that we will implement. It lives inside the ROS2_PX4_Offboard_Example directory.

Before we build these two packages, we need to source our ROS2 installation. Run this code to do that

```
source /opt/ros/humble/setup.bash
```

This will need to be run in every terminal that wants to run ROS2 commands. An easy way to get around this, is to add this command to your .bashrc file. This will run this command every time you open a new terminal window.

To build these two packages, you must be in workspace directory not in src, run this code to change directory from src to one step back i.e. root of your workspace and build the packages

```
cd ..
colcon build
```
As mentioned in Jaeyoung Lim's [example](https://github.com/Jaeyoung-Lim/px4-offboard/blob/master/doc/ROS2_PX4_Offboard_Tutorial.md) you will get some warnings about setup.py but as long as there are no errors, you should be good to go.


After this runs, we should never need to build px4_msgs again. However, we will need to build px4_offboard every time we make changes to the code. To do this, and save time, we can run
```
colcon build --packages-select px4_offboard
```

If you tried to run our code now, it would not work. This is because we need to source our current workspace. This is always done after a build. To do this, be sure you are in the src directory, and then run this code

```
source install/setup.bash
```

We will run this every time we build. It will also need to be run in every terminal that we want to run ROS2 commands in.


### Running the Code
This example has been designed to run from one launch file that will start all the necessary nodes. The launch file will run a python script that uses gnome terminal to open a new terminal window for MicroDDS and Gazebo.

Run this code to start the example

```
ros2 launch px4_offboard offboard_velocity_control.launch.py
```


## Course Packages

The repository contains the following packages:

* `ros2_essential_cpp`: Covers essential ROS2 concepts and examples using the C++ language.
* `ros2_essential_python`: Explores essential ROS2 concepts and examples using the Python language.
* `ros2_interfaces_cpp`: Provides tutorials on defining and using custom ROS2 interfaces with the C++ language.
* `ros2_motion_cpp`: Offers tutorials on robot motion control using ROS2 and the C++ language.
* `ros2_motion_python`: Presents tutorials on robot motion control using ROS2 and the Python language.
* `ros2_laser_python`: Includes tutorials on working with laser scanners in ROS2 using Python.

## Getting Started

Before using the packages provided in this repository, ensure that ROS2 is installed on your system. The [ROS2 installation guide](https://index.ros.org/doc/ros2/Installation/) offers detailed instructions on how to install ROS2.

### Cloning the Repository

Follow these steps to clone the repository and set up the course packages on your local machine:

1. Open a terminal and navigate to the `src` directory of your ROS2 workspace:

   ```bash
   cd ~/ros2_ws/src
   ```

2. Clone this Git repository into the `src` directory:

   ```bash
   git clone https://github.com/riotu-lab/ros2_course_packages.git
   ```

### Configuring the Workspace

After cloning the repository, you need to configure your workspace:

1. Move the content of the `ros2_course_packages` folder to the `src` directory:

   ```bash
   cd ~/ros2_ws/
   mv src/ros2_course_packages/* src/
   ```

2. Remove the now empty `ros2_course_packages` folder:

   ```bash
   rm -r src/ros2_course_packages
   ```

Your ROS2 workspace is now configured with the course packages.

### Building and Running the Packages

With the packages set up in your ROS2 workspace, you can now build and run them. Refer to the specific package documentation for instructions on how to build and execute the provided examples.

Remember to source your ROS2 workspace before running any examples:

```bash
source ~/ros2_ws/install/setup.bash
```

## Running Examples

To run the examples provided in each package, follow the steps below:

1. Build the packages by navigating to the root of your ROS2 workspace and running the `colcon build` command:

   ```bash
   cd ~/ros2_ws
   colcon build
   ```

2. Source the installed packages to make them available for use:

   ```bash
   source install/setup.bash
   ```

3. Execute any example provided in the packages using the `ros2 run` command. For instance, to run the `talker` node from the `ros2_essential_cpp` package, use the following command:

   ```bash
   ros2 run ros2_essential_cpp talker
   ```

Refer to each package's documentation for specific instructions on how to run its examples.

## Maintainers

This repository is maintained by Prof. Anis Koubaa. If you have any questions or suggestions, please feel free to [contact us](mailto:email@example.com).

## Contributing

Contributions are welcome! If you find any issues with the packages or would like to contribute, please create a new [issue](https://github.com/riotu-lab/ros2_course_packages/issues) or [pull request](https://github.com/riotu-lab/ros2_course_packages/pulls) on the GitHub repository.

## License

This project is licensed under the [Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International License](https://creativecommons.org/licenses/by-nc-sa/4.0/).

## Enroll in the Course

To enroll in the Udemy course, visit [ROS2 For Beginners](https://www.udemy.com/ros2-how-to/). 
For discount coupons, check [https://www.riotu-lab.org/udemy.php](https://www.riotu-lab.org/udemy.php).
