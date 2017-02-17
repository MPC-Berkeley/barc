#!/usr/bin/env sh


# necessary for virtual environment
source ~/barc/scripts/startup.sh

# general env setup for ROS
source /opt/ros/indigo/setup.sh

# catkin workspace setup
source ~/barc/workspace/devel/setup.bash

host=$(hostname)

echo -n "What is the hostname of the master? > "
read master

export ROS_MASTER_URI=http://${master}.local:11311
export ROS_IP=${host}.local
export ROS_HOSTNAME=${host}.local
exec "$@"
