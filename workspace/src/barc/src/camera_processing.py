#!/usr/bin/env python

import roslib
import sys
import rospy
import cv2
from std_msgs.msg import String, Int32, Float32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from scipy import signal
from math import sqrt, atan, pi
# import kernel
from data_service.msg import TimeData
import time
from barc.msg import ECU, Encoder
import serial
from numpy import zeros, hstack, cos, array, dot, arctan, sign, uint8, float64
from numpy import unwrap, diff
from scipy import signal

def shutdown_function():
  print "camera_processing.py shutting down!"

# ===================== PARAMETERS FOR IMAGE PROCESSING ================================== #
# mtx matrix 1
dist_mtx = array([[ 122.50633074,    0.        ,  178.97367156],
       [   0.        ,  163.00785794,  104.0821372 ],
       [   0.        ,    0.        ,    1.        ]])


# dist vector 2
dist_vec = array([[-0.25830383,  0.05052929,  0.00036932, -0.00759431,  0.00311647]])

# new mtx matrix 3
new_dist_mtx = array([[  87.04174042,    0.        ,  173.43726247],
       [   0.        ,  112.91073608,  104.21146521],
       [   0.        ,    0.        ,    1.        ]])

# roi vector 4
roi = (10, 22, 304, 204)

def camera_callback(data):
    global cv_image
    global bridge
    global dist_mtx, dist_vec, new_dist_mtx, roi

    img_pub = rospy.Publisher('img_corrected', Image , queue_size = 10)

    bridge = CvBridge()
    # Convert ROS Image Message to CV2 image
    try:
        cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
        print(e)


    #backtorgb  = cv2.cvtColor(gray_image, cv2.COLOR_GRAY2RGB)

    # undistorting image
    undist_img = cv2.undistort(cv_image, dist_mtx, dist_vec, None, new_dist_mtx)
    x,y,w,h    = roi
    undist_img = undist_img[y:y+h, x:x+w]

    # gray scale
    # gray_image = cv2.cvtColor(undist_img, cv2.COLOR_BGR2GRAY)
    # backtorgb = cv2.cvtColor(gray_image, cv2.COLOR_GRAY2RGB)

    img_pub.publish(bridge.cv2_to_imgmsg(undist_img, "bgr8"))

def main_auto():
    global cv_image
    global bridge

    # initialize ROS node
    rospy.init_node('camera_save', anonymous=True)
    rospy.Subscriber("/barc_cam/image_raw", Image, camera_callback)

    rateHz  = 50
    dt      = 1.0 / rateHz
    rate    = rospy.Rate(rateHz)

    rospy.on_shutdown(shutdown_function)

    # main loop
    while not rospy.is_shutdown():
      # wait
      rate.sleep()

if __name__ == '__main__':
    try:
        main_auto()
    except rospy.ROSInterruptException:
        pass
