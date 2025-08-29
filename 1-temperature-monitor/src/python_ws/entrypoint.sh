#!/bin/bash

# this tells to shell to exit if any command returns a non zero exit status 
# non-zero error is an indication of failure in Unix systems
set -e

# this two commands will source the overlay and underlay
# such that we can run our package 
source /opt/ros/humble/setup.bash
source /temp_publisher/python_ws/install/setup.bash
export ROS_DOMAIN_ID=4
ros2 run temp_publisher publisher
#ros2 run temp_publisher temp
# This means that we are doing everything in this  entrypoint.sh script, 
# then in the same shell, we will run the command the user passes in on the command line.
# (if the use passes a command)
exec "$@"