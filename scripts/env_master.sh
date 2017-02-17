#!/bin/bash

source ~/barc/scripts/startup.sh
source /opt/ros/indigo/setup.sh
source ~/barc/workspace/devel/setup.bash


host=$(hostname)
                                                                                                                       
export ROS_MASTER_URI=http://${host}.local:11311
export ROS_IP=${host}.local
export ROS_HOSTNAME=${host}.local
exec "$@"
