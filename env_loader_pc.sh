#!/bin/bash
source ~/barc/scripts/startup.sh
source /opt/ros/kinetic/setup.bash
source /home/shuqi/barc/workspace/devel/setup.bash
#export PATH=$PATH:/home/ugo/julia/bin/julia
export ROS_IP=127.0.0.1
export ROS_MASTER_URI=http://127.0.0.1:11311
exec "$@"
