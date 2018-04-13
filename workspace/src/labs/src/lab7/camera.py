#!/usr/bin/env python

# Author: Tony Zheng
# ME 131 Lab 7

import rospy
import time
import roslib
import sys
import cv2
import numpy as np

# state estimation node
class camera_node():
    def __init__(self):    
        
        self.vid = cv2.VideoCapture("/dev/video6")
        self.vid.set(12,5) #contrast
        self.vid.set(13,0) #saturation

        # Calibration Matrices
        self.mtx = np.array([[592.156892, 0.000000, 326.689246], [0.000000, 584.923917, 282.822026], [0.000000, 0.000000, 1.000000]])
        self.dist = np.array([-0.585868, 0.248490, -0.023236, -0.002907, 0.000000])

        # Camera resolution
        self.w = 640
        self.h = 480

        # Set node rate
        self.loop_rate   = 30
        self.ts          = 1.0 / self.loop_rate
        self.rate        = rospy.Rate(self.loop_rate)
        self.t0          = time.time()

        # Compute the udistortion and rectification transformation map
        self.newcameramtx, self.roi = cv2.getOptimalNewCameraMatrix(self.mtx,self.dist,(self.w,self.h),0,(self.w,self.h))
        self.mapx,self.mapy = cv2.initUndistortRectifyMap(self.mtx,self.dist,None,self.newcameramtx,(self.w,self.h),5)

	while not rospy.is_shutdown():
            self.rel,self.dst = self.vid.read() # gets the current frame from the camera

            self.cv_image = cv2.remap(self.dst,self.mapx,self.mapy,cv2.INTER_LINEAR) #Undistorts the fisheye image to rectangular
            self.x,self.y,self.w,self.h = self.roi
            self.dst = self.dst[self.y:self.y+self.h, self.x:self.x+self.w]
            cv2.imshow('Camera View',self.cv_image)

            # Waitkey is necesarry to update image
            cv2.waitKey(3)
 
            self.rate.sleep()


def shutdown_func():
    cv2.destroyAllWindows()

def main(args):
    rospy.on_shutdown(shutdown_func)
    global camera_global

    # Intialize the node
    rospy.init_node('camera_node', anonymous=True)

    camera_global = camera_node()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        main(sys.argv)
    except rospy.ROSInterruptException:
        pass
