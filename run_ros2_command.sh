#!/bin/bash
export ROS_DISTRO=iron
export ROS_ROOT=/opt/ros/iron
export ROS2_WS=/home/bigchungus/ros2_iron_ws  # Set this to your workspace path

# Temporarily remove MATLAB from PATH and LD_LIBRARY_PATH
OLD_PATH=$PATH
OLD_LD_LIBRARY_PATH=$LD_LIBRARY_PATH
export PATH=$(echo $PATH | sed 's|/usr/local/MATLAB/[^:]*:||g')
export LD_LIBRARY_PATH=$(echo $LD_LIBRARY_PATH | sed 's|/usr/local/MATLAB/[^:]*:||g')

source /opt/ros/iron/setup.bash
source $ROS2_WS/install/setup.bash

# Add ROS 2 libraries to the beginning of LD_LIBRARY_PATH
export LD_LIBRARY_PATH=/opt/ros/iron/lib:$LD_LIBRARY_PATH

# Ensure ROS_DOMAIN_ID is set to a valid integer
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}

# Change to the workspace directory
cd $ROS2_WS

# Define the ROS 2 command with arguments
command="ros2 run ur3e_control_package move_command"
args="--ros-args -p initial_move:=False -p waypoints=\"'[[[0.30, 0.14, 0.25], [0.5, 0.5, 0.5, 0.5]]]\"\" -p velocity:=0.02 -p dt:=0.2"

# Print the command for debugging
echo "Command to run: $command $args"

# Execute the command
if [ "$1" = "bg" ]; then
    $command $args &
else
    $command $args
fi

# Restore original PATH and LD_LIBRARY_PATH
export PATH=$OLD_PATH
export LD_LIBRARY_PATH=$OLD_LD_LIBRARY_PATH