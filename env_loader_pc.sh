#!/bin/bash
source /opt/ros/kinetic/setup.bash                  # set-up system-wide ROS gloval environment variables (kinetic version)
source $HOME/barc/workspace/devel/setup.bash        # set-up ROS environment variables within workspace
export PATH=$PATH:~julia-0.5/bin/                   # set-up link to Julia programming language
export ROS_IP=192.168.100.72                        # set IP address of this machine (laptop)
export ROS_MASTER_URI=http://192.168.100.72:11311   # set IP address of master machine 
exec "$@"                                           # redirect input arguments
