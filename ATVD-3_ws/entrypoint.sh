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

# Execute the command passed into this entrypoint
exec "$@"
