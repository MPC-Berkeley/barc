#!/usr/bin/env python

import rospy

import random

from std_msgs.msg import Float64
from geometry_msgs.msg import Vector3

if __name__ == '__main__':
    # Test rosbag

    rospy.init_node('barc_gui_test', anonymous=True)

    # pub = rospy.Publisher('barc_rosbag', Float64, queue_size=10)
    pub = rospy.Publisher('enc_data', Vector3, queue_size=10)
    # pub2 = rospy.Publisher('imu_data', Float64, queue_size=10)
    rate = rospy.Rate(100)

    while not rospy.is_shutdown():
        pub.publish(Vector3(random.random(), random.random(), random.random()))
        # pub2.publish(random.random())
        rate.sleep()
