# UR3e Arm with ROS2 Setup and Usage Guide

## Overview
This repository serves as the foundation for all tutorial and demonstration code for Duke's UR3e robot platform. This guide will walk you through setting up a UR3e robotic arm with ROS2 on an Ubuntu system. By the end, you will be able to control and simulate the UR3e arm using ROS2.

## Prerequisites
- A computer running Ubuntu 22.04. If you're on Windows, install a Linux subsystem by opening a terminal and typing `wsl --install`. After installation, you can open Ubuntu terminals by searching "Ubuntu" in the Windows search bar.
- Basic command line operation knowledge.
- Internet connection for downloading packages.

# Tutorial Part 1, Setup and Installation for Running a Demo on the UR3E

## Installation and Setup

### Step 1: Install Essential Software Packages

1. **Install software-properties-common**
    ```sh
    sudo apt install software-properties-common
    ```
   For general first time setup. This package provides an abstraction of the used apt repositories. It allows you to easily manage your distribution and independent software vendor software sources. 

2. **Add the universe repository**
    ```sh
    sudo add-apt-repository universe
    ```
   For general first time setup. The universe repository contains open-source software maintained by the community, which is necessary for many ROS2 packages.

3. **Update package list and install curl**
    ```sh
    sudo apt update && sudo apt install curl -y
    ```
   For general first time setup. `curl` is a command-line tool for transferring data with URLs, which is needed for downloading external packages.

### Step 2: Set Up ROS2 Repository

1. **Add ROS2 GPG key**
    ```sh
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
    ```
   This GPG key makes sure the ROS packages you download are authenticated and have not been tampered with.

2. **Add ROS2 repository to your sources list**
    ```sh
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
    ```
   This command adds the ROS2 repository to your system, so you can install ROS2 packages via apt.

### Step 3: Install ROS2 and Necessary Tools

1. **Update package list and install ROS development tools**
    ```sh
    sudo apt update && sudo apt install ros-dev-tools
    ```
   ROS development tools include utilities for building and managing ROS2 packages.

2. **Update package list and upgrade existing packages**
    ```sh
    sudo apt update && sudo apt upgrade -y
    ```
   

3. **Install ROS2 desktop**
    ```sh
    sudo apt install ros-iron-desktop
    ```
   This installs the full ROS2 desktop environment, including all necessary tools and libraries for development.

4. **Install Visual Studio Code (VScode)**
    ```sh
    wget -q https://packages.microsoft.com/keys/microsoft.asc -O- | sudo apt-key add -
    sudo add-apt-repository "deb [arch=amd64] https://packages.microsoft.com/repos/vscode stable main"
    sudo apt update
    sudo apt install code
    ```
   Visual Studio Code makes navigating and editing files in linux much easier.

5. **Install UR Robot Driver**
    ```sh
    sudo apt-get install ros-iron-ur
    ```
   This package contains the necessary drivers to interface with UR robots within ROS2.

### Step 4: Install Docker (ONLY FOR URSIM, SKIP IF YOU'RE USING GAZEBO OR REAL ROBOT)

1. **Install necessary packages for Docker**
    ```sh
    sudo apt install apt-transport-https ca-certificates curl software-properties-common
    ```
   These packages are required for downloading and installing Docker.

2. **Add Docker's official GPG key and repository**
    ```sh
    curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo apt-key add -
    sudo add-apt-repository "deb [arch=amd64] https://download.docker.com/linux/ubuntu focal stable"
    ```
   Adding Docker’s GPG key and repository to get the official Docker packages.

3. **Install Docker CE**
    ```sh
    sudo apt update && sudo apt install docker-ce
    ```
   Docker CE allows you to run URSIM

4. **Add your user to the Docker group**
    ```sh
    sudo usermod -aG docker $USER
    exec su -l $USER
    ```
   Adding your user to the Docker group allows you to run Docker commands without needing `sudo`.

### Step 5: Clone Necessary Repositories

1. **Create a ROS2 workspace and source directory**
    ```sh
    mkdir -p ~/ros2_iron_ws/src
    cd ~/ros2_iron_ws/src
    source /opt/ros/iron/setup.bash
    ```
   Creating a workspace and sourcing the directory for the ROS2 project.

2. **Clone the URSim, PyMoveit2, and ur3e codebase GitHub repositories**
    ```sh
    git clone https://github.com/will-d-chen/Universal_Robots_ROS2_Gazebo_Simulation.git
    git clone https://github.com/will-d-chen/pymoveit2.git
    git clone https://github.com/will-d-chen/ur3e-codebase.git
    ```
   These repositories contain packages necessary to simulate and control the UR3e robot.

### Step 6: Install Dependencies and Build Packages

1. **Install necessary dependencies**
    ```sh
    rosdep init
    rosdep update && rosdep install --ignore-src --from-paths . -y
    ```
   `rosdep` installs system dependencies required by your ROS2 packages.

2. **Build and activate ROS2 packages**
    ```sh
    colcon build
    source install/setup.bash
    ```
   `colcon` is the command-line tool used to build ROS2 packages, and sourcing the setup script activates the environment.

### Step 7: Install MATLAB for Linux (if needed)

1. **Download MATLAB** (if needed)
    ```sh
    https://www.mathworks.com/downloads/
    ```
   Download MATLAB from the official website.

2. **Follow the Installation Instructions** (if needed)
    ```sh
    https://www.mathworks.com/help/install/ug/install-products-with-internet-connection.html
    ```
   Follow the instructions to install MATLAB and required toolboxes: the Robotics System Toolbox and the ROS Toolbox.

3. **Open the Wrapper Script for Executing ROS2 commands in MATLAB**
    ```sh
    cd ~/ros2_iron_ws/src/ur3e-codebase
    nano run_ros2_command.sh
    ```
   This script makes it possible to execute ROS2 commands within MATLAB.

4. **Edit the Wrapper Script** 
   Change USERNAME in line 4 to yours, then hit `Ctrl + S` and `Ctrl + X` to save and close the file.

5. **Make the wrapper script executable**
   In the terminal, run:
    ```sh
    chmod +x run_ros2_command.sh
    ```

## Start the Environment (Choose ONE option)

### Option 1: URSim 

#### Step 1: Run UR3e Simulation

1. **Open a new terminal and source the ROS2 setup script**
    ```sh
    source /opt/ros/iron/setup.bash
    ```

2. **Start the URSim simulator**
    ```sh
    ros2 run ur_client_library start_ursim.sh -m ur3e
    ```

#### Step 2: Launch UR3e Driver and Visualization (URSim)


1. **Open the Polyscope interface**
    - If you are on WSL, open a separate terminal and type `firefox`, install it if you haven't yet
    ```sh
    http://192.168.56.101:6080/vnc.html
    ```

2. **Setup the robot for external control**
    - Inside Polyscope:
      1. Click "Program the Robot", then click the red button on the bottom left; if this is your first time starting, click "confirm configuration" on the pop-up.
      2. Click "On".
      3. Click "Start" when it becomes available.
      4. Click "Exit".
      5. Navigate to "Program" -> Urcaps.
      6. Click on "External Control" once.


3. **Launch the UR3e driver and RViz**
    ```sh
    source /opt/ros/iron/setup.bash
    ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur3e robot_ip:=192.168.56.101 launch_rviz:=true
    ```
4. **Setup the robot for external control**
    - Go back inside Polyscope:
      1. Click on the start/pause button on the bottom right of the screen, on the left of "Simulation".
      2. Click on "Play from selection #: Control by .....".

### Option 2: Gazebo Classic

#### Step 1: Launch Gazebo and Moveit

1. **Ensure your workspace is built and activated**
    ```sh
    cd ~/ros2_iron_ws
    source install/setup.bash
    ```

2. **Launch Gazebo and Moveit**
    ```sh
    ros2 launch ur_simulation_gazebo ur_sim_moveit.launch.py
    ```
    **Note:** The launch defaults to a ur5e, make sure to go into the launch files and edit them to default to ur3e.

### Option 3: Real Robot

#### Step 1: Launch UR3e Driver and Visualization (Real Robot)


1. **Setup the robot for external control**
    - On the robot tablet:
      1. Press the red button (power off) on the bottom left; if this is your first time starting, click "confirm configuration" on the pop-up.
      2. Press "On".
      3. Press "Start" when it becomes available.
      4. Press "Exit".
      5. Navigate to "Program" -> Urcaps.
      6. Press on "External Control" once.


2. **Launch the UR3e driver and RViz**
    ```sh
    source /opt/ros/iron/setup.bash
    ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur3e robot_ip:=192.168.56.101 launch_rviz:=true
    ```
3. **Setup the robot for external control**
    - Go back to the tablet:
      1. Press on the start/pause button on the bottom right of the screen, on the left of "Simulation".
      2. Press on "Play from selection #: Control by Desktop".

4. **AFTER you finish with the robot**
    - Go back to the tablet:
      1. Press the green button on the bottom left (normal)
      2. Press on the red "Off" button
      3. Press the metal on/off button on the tablet to shut down, click "do not save" if prompted



## Executing the Code

### Step 2: Build and Source

1. **In a new terminal**
    ```sh
    source /opt/ros/iron/setup.bash
    colcon build
    source install/setup.bash
    ```


### Step 2: Launch Moveit2

1. **If using URSIM or the real robot**
    - Complete the corresponding steps from Option 1 or Option 3.
    ```sh
    ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur3e launch_rviz:=true
    ```

2. **If using Gazebo**
    - If you completed the corresponding steps from Option 2, it should already be running, if not.
    ```sh
    ros2 launch ur_simulation_gazebo ur_sim_moveit.launch.py
    ```

### Step 3: Launch a Demo

1. **In a new terminal, repeat step 1, then finally, run your package**
    ```sh
    ros2 run ur3e_control_package initials_demo
    ```

## Using MATLAB

### Step 1: Launch MATLAB

1. **Launch MATLAB**
    - Launch MATLAB through the UI or by typing the command `matlab` in the terminal. If you need to install the toolboxes, use `sudo matlab` to start MATLAB initially.
    - Navigate to /home/user/ros2_iron_ws/src/ur3e-codebase within MATLAB.
    - Open `run_ros2.m`
    - Run the script, running section by section is recommended


# Tutorial Part 2: Understanding and Using the Custom 'move_command' Code for Robot Control

This tutorial will guide you through using the custom 'move_command' code to control the UR3e robot. We'll cover the basics of ROS 2 (Robot Operating System 2) and how this code interacts with a robotic system.

## Prerequisites

1. Basic understanding of Python programming
2. Everything in part 1 installed on your system

## 1. Understanding ROS 2 Basics

ROS 2 is a framework for writing robot software. It consists of:

- Nodes: Processes that perform computation
- Topics: Buses over which nodes exchange messages
- Messages: ROS data type used when subscribing or publishing to a topic
- Parameters: Configuration values for nodes

## 2. Examples of ROS 2 Basics

After you launch the necessary launch files, you can open a new terminal, source ROS2, and use this command to see all the broadcasting topics
    ```
    ros2 topic list
    ```
If you want to access one of the topics, you can use this command to have ROS2 return the currently broadcasting values, joint states for example:
    ```
    ros2 topic echo /joint_states
    ```
All the topics can be accessed within python scripts as well. For example, in the move_command file, the joint states are automatically being accessed and saved in a csv file, in the directory that you are executing the code from.
## 3. Code Overview

The `DynamicTrajectoryExecutor` is a custom ROS 2 node inside 'move_command' that:

1. Plans and executes trajectories for a UR3e robot
2. Logs joint states and force/torque data
3. Handles collision objects in the robot's environment

## 4. Running the Code

The code can be run using the `ros2 run` command with various parameters. Here are some example commands:

1. Move to initial position in joint space (Recommended):
    ```
    ros2 run ur3e_control_package move_command --ros-args -p initial_move:=True -p initial_joint_pos:="[-1.9991989999999866, -1.835606000000002, -2.0968710000000046, -2.349238999999997, -0.4251269999999927, -0.0012429999999703512]" -p velocity:=0.02 -p dt:=0.2

    ```
2. Execute a trajectory in world frame in task space:
    ```
    ros2 run ur3e_control_package move_command --ros-args -p initial_move:=False -p waypoints:="'[[[0.30, 0.14, 0.25], [0.5, 0.5, 0.5, 0.5]]]'" -p velocity:=0.02 -p dt:=0.2

    ```
3. Execute a trajectory in end-effector frame in task space:
    ```
    ros2 run ur3e_control_package move_command --ros-args -p initial_move:=False -p waypoints:="'[[[0.0,0.1,0.0], [0.0,0.0,0.0,1.0]]]'" -p frame:="ef" -p velocity:=0.02 -p dt:=0.2

    ```

If you decide to not run from the command line, you can also go into the script to directly edit it to run your movement sequence.

## 4. Understanding the Parameters

- `initial_move`: Boolean to decide if the robot will move to an initial position using joint space control, this makes it safer as you will know exactly where the first joint pose of the robot will be, since task space control will yield different joint configurations, making them less predictable.
- `initial_joint_pos`: Initial joint positions if `initial_move` is True
- `waypoints`: List of positions and orientations for the robot to move through
- `frame`: Coordinate frame for the waypoints (default is "world", declare "ef" for end-effector)
- `velocity`: Velocity in m/s that the robot will move it's end effector at
- `real_robot`: Boolean to determine whether to run it on the real system or in simulation
- `dt`: Float to set the bandwidth

## 6. Code Breakdown

### 6.1 Initialization

The `__init__` method sets up the node, declares parameters, and initializes the MoveIt2 interface for robot control.

### 6.2 Parameter Declaration

The `declare_and_get_parameters` method declares all the parameters the node can accept and retrieves their values.

### 6.3 MoveIt2 Setup

The `setup_moveit2` method initializes the MoveIt2 interface, which is used for motion planning and execution.

### 6.4 Data Logging

The `setup_data_logging` method sets up CSV logging for joint states and force/torque data.

### 6.5 Collision Objects

The `setup_collision_objects` method adds collision objects (currently includes a floor and a pen) to the planning scene.

### 6.6 Trajectory Computation and Execution

The `compute_and_execute_trajectory` method computes a trajectory through the given waypoints and executes it.


## Contributors
-Will Chen: will.chen@duke.edu

