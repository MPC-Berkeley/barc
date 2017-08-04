#!/bin/bash
export ROS_IP=10.0.0.14
export ROS_MASTER_URI=http://10.0.0.14:11311
exec "$@"
