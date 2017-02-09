#!/bin/bash
source $HOME/barc/scripts/startup.sh                # set up python and misc environment variables 
source /opt/ros/indigo/setup.bash                   # set up system-wide global environment variables on the micro-controller
source $HOME/barc/workspace/devel/setup.bash        # set up ROS environment variables within workspace
export PATH=$PATH:~/julia                           # add Julia programming language to path
export ROS_IP=192.168.100.100                       # set IP address of this machine (micro-controller)
export ROS_MASTER_URI=http://192.168.100.152:11311  # set IP address of master machine
exec "$@"
