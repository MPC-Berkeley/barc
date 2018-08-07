#!/bin/bash

source /opt/ros/kinetic/setup.bash
source /home/sarah/barc/workspace/devel/setup.bash
#export PATH=$PATH:/home/ugo/julia/bin/julia

export ROS_IP=192.168.10.188
export ROS_MASTER_URI=http://192.168.10.188:11311

exec "$@"
