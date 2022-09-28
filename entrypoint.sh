#!/bin/bash
# Basic entrypoint for ROS / Colcon Docker containers

# Source ROS and Colcon workspaces.
source /opt/ros/$ROS_DISTRO/setup.bash
echo "Sourced ROS $ROS_DISTRO workspace."

if [ -f /workspace/devel/setup.bash ]
then
  source /workspace/devel/setup.bash
  echo "Sourced workspace"
fi

# Compile the ROS packages using catkin_make.
if [ -f /workspace/CMakeLists.txt ]
then
  cd /workspace
  catkin build
  echo "Built workspace"
  source /workspace/devel/setup.bash
fi


# # Set environment variables
# export TURTLEBOT3_MODEL=waffle_pi
# export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$(ros2 pkg prefix tb3_worlds)/share/tb3_worlds/models

# Execute the command passed into this entrypoint
exec "$@"
