#!/bin/bash

source /opt/ros/kinetic/setup.bash
source /home/ugo/barc/workspace/devel/setup.bash
#export PATH=$PATH:/home/ugo/julia/bin/julia

export ROS_IP=10.0.1.2
export ROS_MASTER_URI=http://10.0.1.2:11311

exec "$@"
