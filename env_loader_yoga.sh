#!/bin/bash

source ~/barc/scripts/startup.sh
source /opt/ros/kinetic/setup.bash
source ~/barc/workspace/devel/setup.bash

export ROS_IP=192.168.10.74
export ROS_MASTER_URI=http://192.168.10.147:11311

exec "$@"
