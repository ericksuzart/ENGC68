#!/bin/bash
# Basic entrypoint for ROS / Colcon Docker containers

# Source ROS and Colcon workspaces.
source /opt/ros/$ROS_DISTRO/setup.bash

if [ -f /workspace/devel/setup.bash ]
then
  source /workspace/devel/setup.bash
  echo "Sourced ROS $ROS_DISTRO workspace."
fi


# # Set environment variables
# export TURTLEBOT3_MODEL=waffle_pi
# export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$(ros2 pkg prefix tb3_worlds)/share/tb3_worlds/models

# Execute the command passed into this entrypoint
exec "$@"
