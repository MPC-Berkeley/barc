#!/bin/bash
source /opt/ros/kinetic/setup.bash
source /home/mpcubuntu/barc/workspace/devel/setup.bash
#export PATH=$PATH:/home/ugo/julia/bin/julia
export ROS_IP=192.168.10.147
export ROS_MASTER_URI=http://192.168.10.147:11311
exec "$@"
