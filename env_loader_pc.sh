#!/bin/bash
source /opt/ros/kinetic/setup.bash
source /home/mpcubuntu/barc/workspace/devel/setup.bash
#export PATH=$PATH:/home/ugo/julia/bin/julia
export ROS_IP=10.0.0.14
export ROS_MASTER_URI=http://10.0.0.14:11311
exec "$@"
