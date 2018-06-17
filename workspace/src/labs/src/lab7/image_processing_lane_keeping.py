#!/usr/bin/env python

# Author: Tony Zheng

import rospy
import time
import roslib
import sys
import cv2
import scipy.linalg
import numpy as np
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Int32, Float32, Float32MultiArray, Bool, Float64
from sensor_msgs.msg import Image, CompressedImage
from math import sqrt, atan, pi, pow, cos, sin, asin, tan, atan2
from barc.msg import barc_state,ECU, Input, Moving
from cv_bridge import CvBridge

# state estimation node
class image_processing_node():
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

        # Camera resolution
        self.width = rospy.get_param("/width")
        self.height = rospy.get_param("/height")

        # Reference velocity
        self.v_ref = rospy.get_param("/reference_velocity")

        # Number of points for moving average filter
        self.numMovingAveragePoints = rospy.get_param("/numMovingAveragePoints")
        self.movingAverageValue = np.zeros([2,self.numMovingAveragePoints])


        # Number of sample points at the reference velocity to check along the path
        self.numpoints =  rospy.get_param("/numStepsToLookAhead")

        # Set node loop rate (30 hz)
        self.loop_rate   = rospy.get_param("/loop_rate")
        self.dt          = 1.0 / self.loop_rate
        self.rate        = rospy.Rate(self.loop_rate)

        self.f1Matrix = rospy.get_param("/f1Matrix")
        self.f2Matrix = rospy.get_param("/f2Matrix")
        self.bMatrix = rospy.get_param("/bMatrix")
        self.yPixel_to_xInertial_Matrix = rospy.get_param("/yPixel_to_xInertial_Matrix")
        self.xInertial_to_yPixel_Matrix = rospy.get_param("/xInertial_to_yPixel_Matrix")
        self.furthest_distance = rospy.get_param("/furthest_distance")
        self.camera_offset_distance = rospy.get_param("/camera_offset_distance")
        self.flipped_camera = rospy.get_param("/flipped_camera")

        # Compute the udistortion and rectification transformation map
        self.newcameramtx, self.roi     = cv2.getOptimalNewCameraMatrix(self.mtx,self.dist,(self.width,self.height),0,(self.width,self.height))
        self.mapx,self.mapy             = cv2.initUndistortRectifyMap(self.mtx,self.dist,None,self.newcameramtx,(self.width,self.height),5)


        # Messages to be filled
        self.state_constraints = barc_state()
        self.reference_trajectory = barc_state()

        self.bridge = CvBridge()

        # Initialize publishers and subscribers
        self.moving_pub                 = rospy.Publisher("moving", Moving, queue_size=1)
        self.hold_previous_turn_pub     = rospy.Publisher("hold_previous_turn", Bool, queue_size=1)
        self.moving_pub.publish(True)
        self.reference_trajectory_pub   = rospy.Publisher("reference_trajectory", barc_state, queue_size = 1)
        self.reference_image_pub        = rospy.Publisher("image_raw", Image, queue_size = 1)
        self.uOpt_pub                   = rospy.Publisher("uOpt", Input, queue_size=1)
        self.optimal_state_sub          = rospy.Subscriber("optimal_state_trajectory", barc_state, self.convertDistanceToPixels)
        self.dt_pub                     = rospy.Publisher("dt", Float64, queue_size=1)

        # The boolean messages passed to these topics are not used, we only want them for the independently threaded callback function.
        self.draw_lines_pub     = rospy.Publisher("draw_lines", Bool, queue_size=1)
        self.draw_lines_sub     = rospy.Subscriber("draw_lines", Bool, self.draw_lines,queue_size=1)
        self.publish_states_pub = rospy.Publisher("publish_states", Bool, queue_size=1)
        self.publish_states_sub = rospy.Subscriber("publish_states", Bool, self.publish_states,queue_size=1)
        self.show_Image_pub     = rospy.Publisher("show_Image", Bool, queue_size=1)
        self.show_Image_sub     = rospy.Subscriber("show_Image", Bool, self.show_Image,queue_size=1)

        self.count = 0
        self.totalTimeCounter = 0 
        self.totalTime = 0 
        self.averageTime = 0 
        self.publish_image = True;
        self.previousTime = time.time()
        self.printme = False
        self.statepoints=''
        self.camera_distance_calibrated = False
        print("Press Up Arrow to start moving. Press Down Arrow to stop moving.")
        while not rospy.is_shutdown():
            try:
                self.count = self.count +1 # updates the count
                self.rel,self.dst = self.vid.read() # gets the current frame from the camera

                # Updates the sample time
                self.dt = time.time() - self.previousTime 
                self.previousTime = time.time()
                if self.flipped_camera:
                    self.cv_image = cv2.flip(cv2.remap(self.dst,self.mapx,self.mapy,cv2.INTER_LINEAR),-1) #Undistorts the fisheye image to rectangular
                else:
                    self.cv_image = cv2.remap(self.dst,self.mapx,self.mapy,cv2.INTER_LINEAR) #Undistorts the fisheye image to rectangular
                self.x,self.y,self.width,self.height = self.roi

                # colorFilter = True makes the edge detection search for a red/white track using HSV. False will use grayscale and search for any edge regardless of color
                colorFilter =  rospy.get_param("/colorFilter")
                kernel_size = rospy.get_param("/kernel_size")
                if colorFilter:
                    imageToFilter = self.cv_image
                    imageToFilter[0:280,0:self.width] = 0 #blacks out the top portion of the image (not used)

                    #self.hsv = cv2.cvtColor(imageToFilter, cv2.COLOR_BGR2HSV) #.004 

                    # define range of color thresholds in (B,G,R)
                    lower_red = np.flipud(np.array(rospy.get_param("/lower_red")))
                    upper_red = np.flipud(np.array(rospy.get_param("/upper_red")))

                    lower_white = np.flipud(np.array(rospy.get_param("/lower_white")))
                    upper_white = np.flipud(np.array(rospy.get_param("/upper_white")))

                    # Threshold the image to only have the red/white track appear
                    self.reds = cv2.inRange(imageToFilter, lower_red, upper_red)
                    self.whites = cv2.inRange(imageToFilter, lower_white, upper_white) 


                    self.edges = cv2.bitwise_or(self.reds,self.whites) # combines the red filter and white filter images
                    self.edges = cv2.GaussianBlur(self.edges,(kernel_size,kernel_size),0) # blurs the image
                    retval, self.edges = cv2.threshold(self.edges,127,255,cv2.THRESH_BINARY) # converts the blurred greyscale to binary to filter once more

                else:
                    # Convert Color Image to Grayscale
                    gray_image = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2GRAY)
                    gray_image[0:270,0:self.width] = 0
                    gray_image = cv2.GaussianBlur(gray_image, (kernel_size, kernel_size), 0)
                    self.edges = cv2.Canny(gray_image,40,80) # openCV edge detection function

                # Parameters to combine image: dst = alpha*img1+beta*img2+gamma
                alpha = 0.6
                beta = 1
                gamma = 0

                # overlayPointsOnColoredImage = True makes the path show up on top of the colored image. 
                # Draw lanes over image (color or black/white)
                overlayPointsOnColoredImage = rospy.get_param("/overlayPointsOnColoredImage")
                if overlayPointsOnColoredImage:
                    self.line_img_color = np.zeros(self.cv_image.shape, dtype=np.uint8)
                    self.pathOverlayedImage = cv2.addWeighted(self.cv_image,alpha,self.line_img_color,beta,gamma)
                else: 
                    self.edges_color = cv2.cvtColor(self.edges, cv2.COLOR_GRAY2RGB)
                    self.line_img_color = np.zeros(self.edges_color.shape, dtype=np.uint8)
                    self.pathOverlayedImage = cv2.addWeighted(self.edges_color,alpha,self.line_img_color,beta,gamma)

                # Collect 100 images before running image processing
                if self.count>100:
                    self.draw_lines_pub.publish(True)
                
                # Publish image with lanes
                if self.publish_image:
                    if True:
                        self.reference_image_pub.publish(self.bridge.cv2_to_imgmsg(cv2.cvtColor(self.edges,cv2.COLOR_GRAY2RGB), "bgr8"))
                    else:
                        print('Could not publish reference image')

                # Check the true loop rate of incoming images from camera (i.e. frames per second should match parameter specified in launch file)
                if (self.count > 100 and self.count%3==0):
                    self.totalTimeCounter +=1
                    self.timenext = time.time()
                    self.timeElapsed = self.timenext - self.previousTime
                    self.totalTime = self.totalTime+self.timeElapsed
                    self.averageTime = self.totalTime/(self.totalTimeCounter)
                    self.dt_pub.publish(self.averageTime)
                    #print('Average Time: ',self.averageTime)
                self.rate.sleep()

            except IOError, (ErrorNumber, ErrorMessage):
                print('HERE')
                print('HERE')
                print(ErrorMessage)
                pass

    #################################################################################
    def show_Image(self,data):
        #cv2.imshow("Advanced Lane Detection ed", self.edges[270:480,:])
        #cv2.imshow("Advanced Lane Detection", self.pathOverlayedImage)
        #cv2.imshow('cv_image',self.cv_image[270:480,:])
        cv2.imshow("Advanced Lane Detection", self.pathOverlayedImage[270:480,:])
        cv2.waitKey(3) # Waitkey is necesarry to update image

    ##############################################################################################

    def draw_lines(self,data):
        thickness = 3
        color = [0, 0, 255 ]
        img = self.line_img_color
        height, width = self.edges.shape 
        index_x = (width)//2        
        offset = 0
        previous_x = index_x
        previous_y = 0
        endtrack = False

        self.stopMoving = False
        converge_limit = rospy.get_param("/converge_limit") # if the left and rath paths converge within 100 pixels, it indicates an obstacle or dead end which stops the vehicle
        i=0
        dt = self.averageTime
        y_base = -1

        for k in xrange(1,self.numpoints+1,1):
            # Starting with one time step ahead, finds the pixel corresponding to that distance
            while self.camera_distance_calibrated == False:
                xIforward = (self.v_ref*dt*k)+self.camera_offset_distance
                y_base = int(self.calc_x_Inertial_to_y_newPixel(xIforward))
                if y_base < 1:
                    self.camera_offset_distance = self.camera_offset_distance+0.005
                else:
                    self.camera_distance_calibrated = True
            xIforward = (self.v_ref*dt*k)+self.camera_offset_distance
            y_base = int(self.calc_x_Inertial_to_y_newPixel(xIforward))
            index_y = height - y_base 
            index_x = previous_x

            # finds the lane edges at the x and y value of the pixel
            x_left, x_right,y_left,y_right = self.find_lane_edges(self.edges, index_x, index_y, width)

            if (not(k==1)and (x_right-x_left)<converge_limit):
                # Vehicle stops moving when the lane edges converge. Previous left and right lane edge pixel values are held.
                x_left = leftpts[-1][0]
                x_right = rightpts[-1][0]
                y_left = leftpts[-1][1]
                y_right = rightpts[-1][1]
                self.stopMoving = True

            midpointx = (x_right + x_left)//2
            midpointy = (y_right + y_left)//2

            midpts = np.array([(midpointx,midpointy)],dtype = np.int32)
            leftpts = np.array([(x_left,y_left)],dtype = np.int32)
            rightpts = np.array([(x_right,y_right)],dtype = np.int32)

            # if it is the first longitudal point, the lists are made of the pixel values of the centerlane and lane edges. Otherwise, the pixel values are appended to the array.
            if (k==1):
                midpointlist = midpts
                leftlist = leftpts
                rightlist = rightpts
            else:
                midpointlist = np.concatenate((midpointlist,midpts))
                leftlist = np.concatenate((leftlist,leftpts))
                rightlist = np.concatenate((rightlist,rightpts))

            # Draws circles on the image where the lane edges are.
            if (not(k==1)): 
                cv2.line(self.pathOverlayedImage, (midpointx, midpointy),(previous_x, previous_y), (0,255,255),3)

            cv2.circle(self.pathOverlayedImage, (x_right,y_right), 4, (0,255,255), -1)
            cv2.circle(self.pathOverlayedImage, (x_left,  y_left), 4, (0,255,255), -1)
            cv2.circle(self.pathOverlayedImage, (midpointx,  midpointy), 3, (0,255,255), -1)
            previous_x = midpointx
            previous_y = midpointy

            if self.statepoints:
                # if there is an optimal trajectory found, plots the points
                j = k-1
                if (self.statepoints[1][j]>self.height):
                    self.statepoints[1][j] = self.height-2
                if ((j>0) and self.count > 10):
                    previous_statex = self.statepoints[0][j-1]
                    previous_statey = self.statepoints[1][j-1]
                    """
                    print(self.statepoints[0][j])
                    print(self.statepoints[1][j])
                    print('')"""
                    cv2.line(self.pathOverlayedImage, (self.statepoints[0][j], self.statepoints[1][j]),(previous_statex, previous_statey), (0, 255,0),3)
                cv2.circle(self.pathOverlayedImage, (self.statepoints[0][j],self.statepoints[1][j]), 4, (0, 255,0), -1)
                #self.statepoints = ''
 
            if endtrack:
                break
            i+=1


        self.globalMidpointList = midpointlist
        self.globalLeftTrackPointList = leftlist
        self.globalRightTrackPointList = rightlist
        if np.unique(leftlist[:,0]).shape[0] == 1 or np.unique(rightlist[:,0]).shape[0] == 1:
            self.hold_previous_turn_pub.publish(True)
        else:
            self.hold_previous_turn_pub.publish(False)
        self.show_Image_pub.publish(True)
        self.publish_states_pub.publish(True)

#################################################################################
    def find_lane_edges(self,img,x,y,width):
        """
        Finds the edge of the track by searching starting from the center of the image and towards the edge along that row of pixels. Also removes some noise with a custom filter
        """
        leftempty = True;
        rightempty = True;
        y_left = y
        y_right = y
        boxsize = rospy.get_param("/boxsize") # checks a box of +-boxsize points around the pixel to see if there are any breaks
        while leftempty:
            xleftarray = np.arange(x) # number range from 0 to the center point
            x_left_index = np.where(img[y_left,xleftarray]>0) #finds the values along a certain row where the pixel value >0 which indicates the track
            try:
                i = -1
                while leftempty:
                    # starts from the last value in the row to see if the pixel is noise or part of the track by checking for continuity along the edges of the box
                    x_left = xleftarray[x_left_index[0][i]] 
                    leftbound = x_left-boxsize
                    if leftbound <0:
                        leftbound=0
                    
                    Top = img[y_left-boxsize,np.arange(leftbound,x_left+boxsize)]
                    Bottom = img[y_left+boxsize,np.arange(leftbound,x_left+boxsize)]
                    Right = img[np.arange(y_left-boxsize,y_left+boxsize),x_left+boxsize]
                    Left = img[np.arange(y_left-boxsize,y_left+boxsize),leftbound]

                    # if the box around the pixel does not have any bright values along all four edges, this is likely a noisy pixel rather the track
                    if (all(Top==0) and all(Bottom==0) and all(Right==0) and all(Left==0)): 
                        i-=1
                    else:
                        leftempty = False;
            except:
                x_left = 0
                leftempty = False;
        while rightempty:
            xrightarray = np.arange(x,width) # number range from the center point to the right edge of the image
            x_right_index = np.where(img[y_right,xrightarray]>0)
            try:
                i = 0
                while rightempty:
                    # starts from the first value in the row to see if the pixel is noise or part of the track by checking for continuity along the edges of the box
                    x_right = xrightarray[x_right_index[0][i]]
                    rightbound = x_right+boxsize
                    if rightbound >=self.width:
                        rightbound=self.width-1
                    Top = img[y_right-boxsize,np.arange(x_right-boxsize,rightbound)]
                    Bottom = img[y_right+boxsize,np.arange(x_right-boxsize,rightbound)]
                    Right = img[np.arange(y_right-boxsize,y_right+boxsize),rightbound]
                    Left = img[np.arange(y_right-boxsize,y_right+boxsize),x_right-10]

                    # if the box around the pixel does not have any bright values along all four edges, this is likely a noisy pixel rather the track
                    if (all(Top==0) and all(Bottom==0) and all(Right==0) and all(Left==0)):
                        i+=1
                    else:
                        rightempty = False;
            except:
                x_right = self.width
                rightempty = False;

        return (x_left, x_right,y_left,y_right)

    ######################################################################################

    def publish_states(self,data):
        """ Converts the centerlane from pixel coordinates to inertial coordinates. Then, publishes the reference trajectory.
        """
        midpointlist = self.globalMidpointList 
        leftlist = self.globalLeftTrackPointList
        rightlist =  self.globalRightTrackPointList

        midpointlist[:,0] = midpointlist[:,0]-self.width/2 # Convert x_pixel to x_newpixel
        midpointlist[:,1] = self.height-midpointlist[:,1] # Convert y_pixel to y_newpixel

        if ((self.count%1000 == 1) and self.printme): 
            print("\nReference Trajectory")
            print(midpointlist)

        midlist_x_Inertial,midlist_y_Inertial = self.convertPixelsToDistance(midpointlist)
        self.reference_trajectory.x = midlist_x_Inertial.tolist()
        self.reference_trajectory.y = midlist_y_Inertial.tolist()
        if (midlist_x_Inertial[-1] <self.furthest_distance):
            self.stopMoving = True
            self.moving_pub.publish(False)
        else:
            self.moving_pub.publish(True)
            self.stopMoving = False

        if ((self.count%10 == 1) and self.printme):
            print(self.reference_trajectory)
        
        self.reference_trajectory_pub.publish(self.reference_trajectory)

    ######################################################################################

    def convertPixelsToDistance(self,inputarray):
        x_newPixel_list = inputarray[:,0]
        y_newPixel_list = inputarray[:,1]
        transformed_y_Inertial_list = np.float32(x_newPixel_list)
        transformed_x_Inertial_list = np.float32(y_newPixel_list)
        
        for i in np.arange(len(x_newPixel_list)):
            x = x_newPixel_list[i]
            y = y_newPixel_list[i]
           
            transformed_y_Inertial_list[i] = self.calc_x_newPixel_to_y_Inertial(x,y) #number of xpixels from center divided by xpixels per foot
            transformed_x_Inertial_list[i] = self.calc_y_newPixel_to_x_Inertial(y)
        return transformed_x_Inertial_list,transformed_y_Inertial_list

    def calc_x_newPixel_to_y_Inertial(self,x_newPixel,y_newPixel):
        # Transforms the xnewpixel into yinertial frame
        x_Inertial = self.calc_y_newPixel_to_x_Inertial(y_newPixel)
        y_Inertial = (x_newPixel-self.b_eq(x_Inertial))/self.f2(x_Inertial)
        y_newPixelskewed = self.f1(y_Inertial)
        x_Inertial = self.calc_y_newPixel_to_x_Inertial(y_newPixel-y_newPixelskewed)
        y_Inertial = (x_newPixel-self.b_eq(x_Inertial))/self.f2(x_Inertial)
        y_Inertial = -y_Inertial
        return y_Inertial

    # define auxiliary functions for mapping from pixel coordinate to inertial frame coordinate
    # these mapping are 3-rd order polynomials 
    # coefficients from polynomials computed in MATLAB ....
################################################################################
    def f1(self,y_Inertial):
        m1 = np.polyval(self.f1Matrix,y_Inertial)
        return m1

    def f2(self,x_Inertial):
        m2 = np.polyval(self.f2Matrix,x_Inertial)
        return m2

    def b_eq(self,x_Inertial):
        b = np.polyval(self.bMatrix,x_Inertial)
        return b

    def calc_y_newPixel_to_x_Inertial(self,y_newPixel):
        # Transforms the ynewpixel into xinertial frame
        x_Inertial =  np.polyval(self.yPixel_to_xInertial_Matrix,y_newPixel)
        x_Inertial=x_Inertial
        return x_Inertial

    def calc_x_Inertial_to_y_newPixel(self,x_Inertial):
        # Transforms the ynewpixel into xinertial frame
        y_newPixel = np.polyval(self.xInertial_to_yPixel_Matrix,x_Inertial)
        return y_newPixel

    def convertDistanceToPixels(self,inputarray):
        if len(inputarray.x)>0:
            xlist = inputarray.x
            ylist = inputarray.y
            xPixelList = list(xlist)
            yPixelList = list(ylist)
            for i in np.arange(len(xlist)):
                if i == 0:
                    xPixelList[i] = self.width/2
                    yPixelList[i] = self.height-1
                else:
                    x = xlist[i]
                    y = ylist[i]
                    xPixelList[i] = self.width/2-int(self.f2(x+self.camera_offset_distance)*y+self.b_eq(x+self.camera_offset_distance))
                    yPixelList[i] =  self.height-int(self.calc_x_Inertial_to_y_newPixel(x+self.camera_offset_distance))-1
            self.statepoints = (xPixelList,  yPixelList)


def shutdown_func():
    cv2.destroyAllWindows()

def main(args):
    # Intialize the node
    rospy.init_node('image_processing_node', anonymous=True)
    rospy.on_shutdown(shutdown_func)
    
    image_processor_global = image_processing_node()

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
