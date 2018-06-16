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
        # Define camera settings
        # put bright, contrast, saturation, hue into image_processing_param.yaml file
        self.vid = cv2.VideoCapture(rospy.get_param("/videoDevicePath")) # Sets the port /dev/video6 as the video device
        self.vid.set(10, rospy.get_param("/brightness")) # brightness
        self.vid.set(11, rospy.get_param("/contrast")) # contrast
        self.vid.set(12, rospy.get_param("/saturation")) # saturation
        self.vid.set(13, rospy.get_param("/hue")) # hue

        # Decalre calibration matrices to rectify the image

        self.mtx = np.array(rospy.get_param("/mtx"))
        self.dist = np.array(rospy.get_param("/dist"))
        self.flipped_camera = np.array(rospy.get_param("/flipped_camera"))


        # Camera resolution
        self.width = rospy.get_param("/width")
        self.height = rospy.get_param("/height")


        # Set node loop rate (30 hz)
        self.loop_rate   = rospy.get_param("/loop_rate")
        self.dt          = 1.0 / self.loop_rate
        self.rate        = rospy.Rate(self.loop_rate)
        self.t0          = time.time()

        # Compute the udistortion and rectification transformation map
        self.newcameramtx, self.roi     = cv2.getOptimalNewCameraMatrix(self.mtx,self.dist,(self.width,self.height),0,(self.width,self.height))
        self.mapx,self.mapy             = cv2.initUndistortRectifyMap(self.mtx,self.dist,None,self.newcameramtx,(self.width,self.height),5)

	while not rospy.is_shutdown():
            self.rel,self.dst = self.vid.read() # gets the current frame from the camera
            if self.flipped_camera:
                self.cv_image = cv2.flip(cv2.remap(self.dst,self.mapx,self.mapy,cv2.INTER_LINEAR),-1) #Undistorts the fisheye image to rectangular
            else:
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
