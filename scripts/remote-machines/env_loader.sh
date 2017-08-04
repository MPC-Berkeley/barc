#!/bin/bash
export ROS_IP=$(hostname -I)
export ROS_MASTER_URI=http://10.0.0.14:11311
exec "$@"
