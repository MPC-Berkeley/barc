#!/bin/bash
source /opt/ros/kinetic/setup.bash
source /home/ugo/GitHub/barc/workspace/devel/setup.bash
export PATH=$PATH:/home/ugo/julia/bin/julia
export ROS_IP=10.0.0.72
export ROS_MASTER_URI=http://10.0.0.72:11311
exec "$@"