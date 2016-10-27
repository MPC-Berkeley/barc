#!/usr/bin/env python

from __future__ import print_function
import roslib
roslib.load_manifest('barc')
import rospy
#from data_service.msg import TimeData
from barc.msg import ECU
import time
import serial
from std_msgs.msg import String, Float32, Int32
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import sys
from math import sqrt, atan, pi
from scipy import signal
import kernel

t0 = time.time() # initialize time
point_num = 0

# Initialize default parameter Values
display_image = False
publish_image = False
calibrate_transform = False
image_calibrated = True
calibration_pts = None

def print_point_callback(event, x, y, flags, param): # Calibrates the image perspective transform 
	global point_num, image_calibrated, calibration_pts, transformation, image_processor_global
	
	if event == cv2.EVENT_LBUTTONDOWN: # Check if left mouse button is down
		point_num += 1
		if (point_num == 1): # when first click captured
			print('Start from upper left corner and select points in clockwise direction')
			calibration_pts = np.float32([[x + 0.0, y + 0.0]])
		elif (point_num <= 4): # for the remaining three locations add on calibration pts
			calibration_pts = np.append(calibration_pts, np.float32([[x + 0.0, y + 0.0]]), axis = 0)
		
		if (point_num == 4): # when we get all 4 points (TL, TR, BR, BL)
		# after collecting all 4 points of the raw image, we applt the projection transform to get a bird's eye view of the image
			reference_pts = np.float32([[image_processor_global.x_offset, 0], 
				[image_processor_global.x_width + image_processor_global.x_offset,0], 
				[image_processor_global.x_width + image_processor_global.x_offset, image_processor_global.y_height], 
				[image_processor_global.x_offset, image_processor_global.y_height]]) # Bottom Left
				
			image_processor_global.projection_dim = (image_processor_global.x_width + image_processor_global.x_offset * 2, image_processor_global.y_height)
			image_processor_global.projection_transfor = cv2.getPerspectiveTransform (calibration_pts, reference_pts) # Apply projection transform
			image_calibrated = True
			cv2.destroyWindow("Calibrate Image Transform")
			
		display_val = 'Pt{}: x:{} y:{}'.format(point_num, x, y)
		print(display_val) # Display the values of each reference and projected points

	
	
	
def find_offset(image, x, y, width): # Finds the center of the two lanes and checks the distance from the car to the center of the two lanes (in x and y coordinates)
	x_left = x
	x_right = x
	while (x_left > 0 and not image[y, x_left]): # check if left side point is greater than the origin i.e 0
		x_left = x_left - 1
	while (x_right < width and not image[y, x_right]): # check if right side point is less than the width 
		x_right = x_right + 1
	return (x_left, x_right)
	
class image_converter:
# Convert Image from the Camera to cv2 format, grayscale it and apply perspective and Hanning transform. 
# Output is a difference value indicating the difference (offset) of the vehicle from the center of the lane
    
    def __init__(self):
	global display_image, publish_image, calibrate_transform
	global calibration_pts
	
        self.image_pub = rospy.Publisher('image_new', Image, queue_size = 10) # publish the converted image 
	self.lane_offset_pub = rospy.Publisher('lane_offset', Float32, queue_size = 10) # publish the offset
	self.ecu_pub = rospy.Publisher('ecu', ECU, queue_size = 10) # Publish the ecu recordings 
	self.cvfail_pub = rospy.Publisher('cv_abort', Int32, queue_size = 10)
	
        self.bridge = CvBridge ()
        self.image_sub = rospy.Subscriber('/serhad_barc_cam/image_raw', Image, self.camera_callback)
    
    # Get the parameters from the launch file and configure node
	calibrate_transform =	rospy.get_param('/image_processing/calibrate_transform')
	display_image =		rospy.get_param('/image_processing/display_image')
	publish_image =		rospy.get_param('/image_processing/publish_image')
	
	global image_calibrated
	
    # Getting transformation parameters
	self.x_offset = 100
	self.x_width = 200
	self.y_height = 400
	if calibrate_transform:
		image_calibrated = False
		cv2.namedWindow('Calibrate Image Transform')
		cv2.setMouseCallback("Calibrate Image Transform", print_point_callback)
	else:
		image_calibrated = True
		calibration_pts  = np.float32( ([rospy.get_param('/image_processing/upperLeftX'), rospy.get_param('/image_processing/upperLeftY')], \
						[rospy.get_param('/image_processing/upperRightX'),rospy.get_param('/image_processing/upperRightY')], \
						[rospy.get_param('/image_processing/lowerRightX'),rospy.get_param('/image_processing/lowerRightY')],\
						[rospy.get_param('/image_processing/lowerLeftX'), rospy.get_param('/image_processing/lowerLeftY')], ))
	
						
		reference_pts = np.float32( [	[(self.x_offset ), 0],
						[(self.x_width + self.x_offset ), 0],
						[(self.x_width + self.x_offset ), self.y_height],
						[self.x_offset, self.y_height],
						]) 
				    
		self.projection_dim = (self.x_width + self.x_offset * 2, self.y_height)
		self.projection_transform = cv2.getPerspectiveTransform(calibration_pts, reference_pts)
	
	
	## Usage when Black Lane is in Parameters of Launch
	
	self.black_lane = rospy.get_param('/image_processing/black_lane')
	self.kernel = kernel.kernel_get(5, 1 , 11, 5, self.black_lane)
	self.quartile = 95.0
	self.prev_offset = 0
        
    def camera_callback(self, data): #Convert raw image data file from ROS format to cv2
	global display_image, publish_image, image_calibrated
	
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, 'bgr8') # Get data from camera as Image file and convert it to cv2 'mono8' array
        except CvBridgeError as e:
            print(e)
	 
	#imageGray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY) # change color of image form RGB to GrayScale  
	    
	(rows, cols, channels) = cv_image.shape # Get width and height of image i.e 480x640
	gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
	gray_image = cv2.bilateralFilter(gray_image, 11, 55, 11)
	
	if display_image: # Convert Image to gray scale and display
		cv2.imshow('Raw Image', gray_image)
		cv2.waitKey(1)
	
	if image_calibrated: # Perform Inverse Perspective Method
		ipm = cv2.warpPerspective(gray_image, self.projection_transform, self.projection_dim)
		if display_image:
			cv2.imshow('IPM', ipm)
			cv2.waitKey(1)
	# here we apply filtering to the image using kernel
		filtered = signal.fftconvolve(self.kernel, ipm)
		threshold = np.percentile(filtered, self.quartile)
		filtered[filtered < threshold] = 0
		filtered[filtered >= threshold] = 255
		filtered = np.array(filtered, dtype = np.uint8)
		
	# Here we convert back to RGB to visualize the lane detection points
		if display_image:
			cv2.imshow('Filtered Image', filtered)
			cv2.waitKey(1)
		if display_image or publish_image:
			backtorgb = cv2.cvtColor( filtered, cv2.COLOR_GRAY2BGR)
			
	
	## Lane Detection
		height, width = filtered.shape # get height and width
		
		
		offset = 0 # initial placement of vehicle at center of lane
		
		
		index_x = width//2 + self.prev_offset
		index_y = 30
		x_left, x_right = find_offset(filtered, index_x, height - index_y, width) # find distance from left and right lanes
		#x_left, x_right = find_offset(edges, index_x, index_y, width) # find distance from left and right lanes
		offset = (x_left + x_right) // 2
		offset_lane = offset - width // 2
		self.prev_offset  = offset_lane
		
		midpoint = (x_right + x_left) // 2
		offset = midpoint - width // 2
		print(self.x_offset, x_right, offset_lane)
		
		if display_image or publish_image:
			cv2.circle(backtorgb, (x_left, height - index_y), 3, (0, 255, 255), -1)
			cv2.circle(backtorgb, (x_right, height - index_y), 3, (0, 255, 255), -1)
			cv2.circle(backtorgb, (offset, height - index_y), 3, (255, 0, 255), -1)
			
			
			fontFace = cv2.FONT_HERSHEY_SCRIPT_SIMPLEX
			cv2.putText(backtorgb, str(offset_lane), (25, index_y - 5), fontFace, .5, (0,0,255))
			
		self.lane_offset_pub.publish(Float32(offset_lane))
		
		if display_image:
			cv2.imshow('final Result', backtorgb)
			cv2.waitKey(1)
		if publish_image:
			try: # Publish the new image to a topic
				self.image_pub.publish(self.bridge.cv2_to_imgmsg(backtorgb, 'bgr8'))
			except CvBridgeError as e:
				print (e)
	else:
		if display_image:
			cv2.imshow('Calibrate Image Transform', gray_image)
		if publish_image:
			try: # Publish the new image to a topic
				self.image_pub.publish(self.bridge.cv2_to_imgmsg(gray_image, 'bgr8'))
			except CvBridgeError as e:
				print (e)
	
	
    if display_image:
		cv2.waitKey(1)

def shutdown_func():
	cv2.destroyAllWindows()
	
image_processor_global = None
       
            
def main():
	rospy.on_shutdown(shutdown_func)
	global image_processor_global
	image_processor_global = image_converter()
	rospy.init_node('image_converter', anonymous = True)
	ic = image_converter()
	
	try:
	    rospy.spin()
	    
	except KeyboardInterrupt:
	    print ("Shutting Down")
	    cv2.destroyAllWindows()
	    
            
if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass
            
            
            
            
            
            
