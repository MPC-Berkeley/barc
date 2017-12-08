#!/bin/bash
source ~/barc/scripts/startup.sh
source /opt/ros/indigo/setup.bash
source ~/barc/workspace/devel/setup.bash
#export PATH=$PATH:/home/odroid/julia
export ROS_IP=10.0.0.1
export ROS_MASTER_URI=http://10.0.0.14:11311
exec "$@"
