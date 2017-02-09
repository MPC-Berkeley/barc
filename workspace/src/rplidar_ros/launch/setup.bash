#!/usr/bin/env bash

# added by jgonzal to initialize virtualenvwrapper on 2016.01.15
source ~/barc/scripts/startup.sh

# added to initialize the ROS environment variables
source /opt/ros/indigo/setup.bash
source ~/barc/workspace/devel/setup.bash
export PATH=$PATH:/home/odroid/julia
# added by jgonzal to set-up communication between multiple machiens
export ROS_MASTER_URI=http://192.168.100.100:11311
export ROS_IP=192.168.100.100
exec "$@"


