# UR3e Arm with ROS2 Setup and Usage Guide

## Overview
This repository serves as the basis for all tutorial/demonstration Code that will be used on Duke's UR3e robot platform. This guide will walk you through the process of setting up a UR3e robotic arm with ROS2 on an Ubuntu system. By the end of this guide, you will be able to control and simulate the UR3e arm using ROS2.

## Prerequisites
- A computer running Ubuntu (22.04 or later recommended).
- Basic knowledge of command line operations.
- Internet connection for downloading packages.

## Installation and Setup

### Step 1: Install Essential Software Packages

1. **Install software-properties-common**
    ```sh
    sudo apt install software-properties-common
    ```

2. **Add the universe repository**
    ```sh
    sudo add-apt-repository universe
    ```

3. **Update package list and install curl**
    ```sh
    sudo apt update && sudo apt install curl -y
    ```

### Step 2: Set Up ROS2 Repository

1. **Add ROS2 GPG key**
    ```sh
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
    ```

2. **Add ROS2 repository to your sources list**
    ```sh
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
    ```

### Step 3: Install ROS2 and Necessary Tools

1. **Update package list and install ROS development tools**
    ```sh
    sudo apt update && sudo apt install ros-dev-tools
    ```

2. **Update package list again**
    ```sh
    sudo apt update
    ```

3. **Upgrade existing packages**
    ```sh
    sudo apt upgrade
    ```

4. **Install ROS2 desktop**
    ```sh
    sudo apt install ros-iron-desktop
    ```

5. **Install VScode**
    ```sh
    wget -q https://packages.microsoft.com/keys/microsoft.asc -O- | sudo apt-key add -
    sudo add-apt-repository "deb [arch=amd64] https://packages.microsoft.com/repos/vscode stable main"
    sudo apt update
    sudo apt install code
    ```

6. **Install UR Robot Driver**
    ```sh
    sudo apt-get install ros-iron-ur
    ```

### Step 4: Install Docker (ONLY FOR URSIM, SKIP IF YOU'RE USING GAZEBO OR REAL ROBOT)

1. **Install necessary packages for Docker**
    ```sh
    sudo apt install apt-transport-https ca-certificates curl software-properties-common
    ```

2. **Add Docker's official GPG key**
    ```sh
    curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo apt-key add -
    ```

3. **Add Docker repository to your sources list**
    ```sh
    sudo add-apt-repository "deb [arch=amd64] https://download.docker.com/linux/ubuntu focal stable"
    ```

4. **Install Docker CE (Community Edition)**
    ```sh
    sudo apt install docker-ce
    ```

5. **Check Docker service status**
    ```sh
    sudo systemctl status docker
    ```

6. **Add your user to the Docker group**
    ```sh
    sudo usermod -aG docker YOUR-UBUNTU-USERNAME
    ```

7. **Reload user groups**
    ```sh
    exec su -l YOUR-UBUNTU-USERNAME
    ```

### Step 5: Clone Necessary Repositories

1. **Create a ROS2 workspace and source directory**
    ```sh
    mkdir -p ~/ros2_ws/src
    cd ~/ros2_ws/src
    source /opt/ros/iron/setup.bash
    ```

2. **Clone the URSim GitHub Repository**
    ```sh
    git clone https://github.com/will-d-chen/Universal_Robots_ROS2_Gazebo_Simulation.git
    ```

3. **Clone the PyMoveit2 GitHub Repository**
    ```sh
    git clone https://github.com/will-d-chen/pymoveit2.git
    ```

4. **Clone the Main Package GitHub Repository**
    ```sh
    git clone https://github.com/will-d-chen/move_robot.git
    ```

### Step 6: Install Dependencies and Build Packages

1. **Install necessary dependencies**
    ```sh
    rosdep update && rosdep install --ignore-src --from-paths . -y
    ```

2. **Build and Activate ROS2 packages**
    ```sh
    colcon build
    source install/setup.bash
    ```

## Usage

### Option 1: URSim (Docker)

#### Step 1: Run UR3e Simulation

1. **Source the ROS2 setup script**
    ```sh
    source /opt/ros/iron/setup.bash
    ```

2. **Start the URSim simulator**
    ```sh
    ros2 run ur_client_library start_ursim.sh -m ur3e
    ```

#### Step 2: Launch UR3e Driver and Visualization (URSim)

1. **Launch the UR3e driver and RViz**

    In a new terminal, source the ROS2 setup script (`source /opt/ros/iron/setup.bash`), then paste this command in to launch the UR3e robot driver and start RViz for visualization.
    ```sh
    ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur3e robot_ip:=192.168.56.101 launch_rviz:=true
    ```

2. **Open the Polyscope interface**
    ```sh
    http://192.168.56.101:6080/vnc.html
    ```

3. **Setup the robot for external control**
    - Inside Polyscope:
      1. Click the red button on the bottom left.
      2. Click "On".
      3. Click "Start" when it becomes available.
      4. Click "Exit".
      5. Navigate to "Program" -> Urcaps.
      6. Click on "External Control" once.
      7. Click on the start/pause button on the bottom right of the screen, on the left of "Simulation".
      8. Click on "Play from selection #: Control by .....".

### Option 2: Gazebo Classic

#### Step 1: Launch Gazebo and Moveit

1. **Ensure your workspace is built and activated**
    ```sh
    cd ~/ros2_ws
    source install/setup.bash
    ```

2. **Launch Gazebo and Moveit**
    
    WARNING: The launch defaults to a ur5e, make sure to go into the launch files and edit them to default to ur3e.
    ```sh
    ros2 launch ur_simulation_gazebo ur_sim_moveit.launch.py
    ```

### Option 3: Real Robot

#### Step 1: Launch UR3e Driver and Visualization (Real Robot)

1. **Launch the UR3e driver and RViz**

    In a new terminal, source the ROS2 setup script (`source /opt/ros/iron/setup.bash`), then paste this command in to launch the UR3e robot driver and start RViz for visualization.
    ```sh
    ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur3e robot_ip:=10.236.66.11 launch_rviz:=true
    ```

2. **Setup the robot for external control**
    - On the robot tablet:
      1. Press the metal power button to start up the tablet.
      2. Click the red button on the bottom left.
      3. Click "On".
      4. Click "Start" when it becomes available.
      5. Click "Exit".
      6. Navigate to "Program" -> Urcaps.
      7. Click on "External Control" once.
      8. Click on the start/pause button on the bottom right of the screen, on the left of "Simulation".
      9. Click on "Play from selection #: Control by .....".

## Additional Steps

### Step 1: Launch Moveit2

1. **If using URSIM or the real robot**
    - Complete the corresponding steps from Option 1 or Option 3.
    - Run:
    ```sh
    ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur3e launch_rviz:=true
    ```

2. **If using Gazebo**
    - If you completed the corresponding steps from Option 2, it should already be running, if not.
    - Run:
    ```sh
    ros2 launch ur_simulation_gazebo ur_sim_moveit.launch.py
    ```

### Step 2: Launch Your Package

1. **Finally, run your package**
    ```sh
    ros2 run move_robot move_robot
    ```
