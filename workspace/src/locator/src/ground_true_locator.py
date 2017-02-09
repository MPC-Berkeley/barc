#!/usr/bin/env python
import rospy
from tf.msg import tfMessage
from geometry_msgs.msg import Vector3
import numpy as np
import sys
import tf
listener = None

def locator(ar_tags):
    listener = tf.TransformListener()
    pub = rospy.Publisher('ground_true_pos', Vector3, queue_size=2)
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        try:
            (trans, rot) = listener.lookupTransform(ar_tags['origin'], ar_tags['car'], rospy.Time(0))
        except:
            continue

        cmd = Vector3()
        cmd.x = trans[0]
        cmd.y = trans[1]
        cmd.z = trans[2]
        pub.publish(cmd)


if __name__ == '__main__':
    rospy.init_node('gt_locator')
    if len(sys.argv) < 3:
        print('Use: ground_true_locator.py [ AR tag number for orgin] [ AR tag number for car] ')
        sys.exit()
    ar_tags = {}
    ar_tags['origin'] = 'ar_marker_' + sys.argv[1]
    ar_tags['car'] = 'ar_marker_' + sys.argv[2]
    locator(ar_tags)
    rospy.spin()
