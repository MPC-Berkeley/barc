#!/bin/bash
source /opt/ros/kinetic/setup.bash
source /home/odroid/barc/scripts/startup.sh
export ROS_IP=10.0.0.1
export ROS_MASTER_URI=http://10.0.0.120:11311
exec "$@"
