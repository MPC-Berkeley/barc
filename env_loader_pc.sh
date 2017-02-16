#!/bin/bash
source /opt/ros/kinetic/setup.bash
source /home/micha/Documents/barc/workspace/devel/setup.bash
export PATH=$PATH:/usr/local/bin/julia4
export ROS_IP=192.168.100.53
export ROS_MASTER_URI=http://192.168.100.53:11311
exec "$@"