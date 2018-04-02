#!/usr/bin/env python

# Author: Tony Zheng
# ME 131 Lab 7

import rospy
import time
import roslib
import sys
import cv2
import controlpy
import scipy.linalg
import numpy as np
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Int32, Float32, Float32MultiArray
from sensor_msgs.msg import Image
from math import sqrt, atan, pi, pow, cos, sin, asin, tan, atan2
from barc.msg import barc_state,ECU, Input, Moving
from controlpy import analysis

# state estimation node
class image_processing_node():
    def __init__(self):    
        
        self.vid = cv2.VideoCapture("/dev/video6")
        self.vid.set(12,5) #contrast
        self.vid.set(13,255) #saturation

        # Calibration Matrices
        self.mtx = np.array([[592.156892, 0.000000, 326.689246], [0.000000, 584.923917, 282.822026], [0.000000, 0.000000, 1.000000]])
        self.dist = np.array([-0.585868, 0.248490, -0.023236, -0.002907, 0.000000])

        # Camera resolution
        self.w = 640
        self.h = 480

        # Reference velocity
        self.v_ref = 2

        # Set node rate
        self.loop_rate   = 30
        self.ts          = 1.0 / self.loop_rate
        self.rate        = rospy.Rate(self.loop_rate)
        self.t0          = time.time()

        self.dt = self.ts
        self.count = 0
        self.total = 0 
        self.avg = 0 
        self.total2 = 0 
        self.avg2 = 0 
        self.publish_image = True;
        self.timeprev = time.time()-self.dt
        time.sleep(0.2)
        # Compute the udistortion and rectification transformation map
        self.newcameramtx, self.roi = cv2.getOptimalNewCameraMatrix(self.mtx,self.dist,(self.w,self.h),0,(self.w,self.h))
        self.mapx,self.mapy = cv2.initUndistortRectifyMap(self.mtx,self.dist,None,self.newcameramtx,(self.w,self.h),5)


        self.statepoints = ''
        self.printme = True

        self.state_constraints = barc_state()
        self.reference_trajectory = barc_state()

        # Initialize publishers and subscribers
        self.moving_pub = rospy.Publisher("moving", Moving, queue_size=1)
        self.moving_pub.publish(True)

        self.state_constraints_pub = rospy.Publisher("state_constraints", barc_state, queue_size = 1)
        self.reference_trajectory_pub = rospy.Publisher("reference_trajectory", barc_state, queue_size = 1)
        self.reference_image_pub = rospy.Publisher("image_reference_trajectory", Image, queue_size = 10)
        #self.optimal_state_sub = rospy.Subscriber("optimal_state_trajectory", barc_state, self.convertDistanceToPixels,queue_size=1)
        self.uOpt_pub = rospy.Publisher("uOpt", Input, queue_size=1)

        while not rospy.is_shutdown():
            self.count = self.count +1
            self.rel,self.dst = self.vid.read() # gets the current frame from the camera

            self.dt = time.time() - self.timeprev
            self.timeprev = time.time()

            self.cv_image = cv2.remap(self.dst,self.mapx,self.mapy,cv2.INTER_LINEAR) #Undistorts the fisheye image to rectangular
            self.x,self.y,self.w,self.h = self.roi
            self.dst = self.dst[self.y:self.y+self.h, self.x:self.x+self.w]

            # True makes the edge detection search for a yellow track using HSV. False will use grayscale and search for any edge regardless of color
            yellow = True
            kernel_size = 7
            if yellow:
                cropped = self.cv_image
                cropped[0:280,0:640] = 0
                hsv = cv2.cvtColor(cropped, cv2.COLOR_BGR2HSV)
                #########cv2.imshow('hsv',hsv[270:480,:])

                # define range of blue color in HSV
                lower_yellow = np.array([0,200,100])
                upper_yellow = np.array([50,255,255])

                # Threshold the HSV image to get only blue colors
                edges = cv2.inRange(hsv, lower_yellow, upper_yellow)
                edges = cv2.GaussianBlur(edges,(kernel_size,kernel_size),0)
                ####################cv2.imshow("Edges",edges[270:480,:])
                edgescropped = edges
            else:
                # Convert Color Image to Grayscale
                gray_image = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2GRAY)
                gray_image[0:270,0:640] = 0
                gray_image = cv2.GaussianBlur(gray_image, (kernel_size, kernel_size), 0)
                edges = cv2.Canny(gray_image,40,80)
                edgescropped = edges

            
            alpha = .6
            beta = 1.
            gamma = 0

            # Colored = True makes the path show up on top of the colored image. 
            colored = True
            if colored:
                line_img_color = np.zeros(self.cv_image.shape, dtype=np.uint8)
                midpointlist,leftlist,rightlist = self.draw_lines(line_img_color,edgescropped)
                LinesDrawn2_color = cv2.addWeighted(self.cv_image,alpha,line_img_color,beta,gamma)
            else: 
                edges_color = cv2.cvtColor(edgescropped, cv2.COLOR_GRAY2RGB)
                line_img_color = np.zeros(edges_color.shape, dtype=np.uint8)
                midpointlist,leftlist,rightlist = self.draw_lines(line_img_color,edges)
                LinesDrawn2_color = cv2.addWeighted(edges_color,alpha,line_img_color,beta,gamma)

            fontFace = cv2.FONT_HERSHEY_TRIPLEX
            self.publish_states(midpointlist,leftlist,rightlist)
            cv2.imshow("Advanced Lane Detection", LinesDrawn2_color[270:480,:])

            if self.publish_image:
                try:
                    self.reference_image_pub.publish(self.bridge.cv2_to_imgmsg(LinesDrawn2_color, "bgr8"))
                except:# CvBridgeError as e:
                    pass#print(e)

            self.timenext = time.time()
            self.timeElapsed = self.timenext - self.timeprev
            self.total2 = self.total2+self.timeElapsed
            self.avg2 = self.total2/self.count
            # Waitkey is necesarry to update image
            cv2.waitKey(3)
 
            self.rate.sleep()

    def find_offset_in_lane(self,img,x,y,width):
        """
        Finds the edge of the track by searching starting from the center of the image and towards the edge along that row of pixels. Also removes some noise with a custom filter
        """
        leftempty = True;
        rightempty = True;
        y_left = y
        y_right = y
        boxsize = 20
        while leftempty:
            xleftarray = np.arange(x)
            x_left_index = np.where(img[y_left,xleftarray]>0)
            try:
                i = -1
                while leftempty:
                    x_left = xleftarray[x_left_index[0][i]]
                    leftbound = x_left-boxsize
                    if leftbound <0:
                        leftbound=0
                    
                    Top = img[y_left-boxsize,np.arange(leftbound,x_left+boxsize)]
                    Bottom = img[y_left+boxsize,np.arange(leftbound,x_left+boxsize)]
                    Right = img[np.arange(y_left-boxsize,y_left+boxsize),x_left+boxsize]
                    Left = img[np.arange(y_left-boxsize,y_left+boxsize),leftbound]
                    if (all(Top==0) and all(Bottom==0) and all(Right==0) and all(Left==0)):
                        i-=1
                    else:
                        leftempty = False;
            except:
                x_left = 0
                leftempty = False;
        while rightempty:
            xrightarray = np.arange(x,width)
            x_right_index = np.where(img[y_right,xrightarray]>0)
            try:
                i = 0
                while rightempty:
                    x_right = xrightarray[x_right_index[0][i]]
                    rightbound = x_right+boxsize
                    if rightbound >639:
                        rightbound=639
                    Top = img[y_right-boxsize,np.arange(x_right-boxsize,rightbound)]
                    Bottom = img[y_right+boxsize,np.arange(x_right-boxsize,rightbound)]
                    Right = img[np.arange(y_right-boxsize,y_right+boxsize),rightbound]
                    Left = img[np.arange(y_right-boxsize,y_right+boxsize),x_right-10]
                    if (all(Top==0) and all(Bottom==0) and all(Right==0) and all(Left==0)):
                        i+=1
                    else:
                        rightempty = False;
            except:
                x_right = 639
                rightempty = False;
        return (x_left, x_right,y_left,y_right)

    def draw_lines(self,img, edges, color=[0, 0,255 ], thickness=3):
        height, width = edges.shape 
        index_x = (width)//2        
        offset = 0
        previous_x = index_x


        ymin = 20 # Where to start the reference trajectory
        ymax = 171 # Where to end the reference trajectory

        previous_y = height-ymin
        endtrack = False

        # Number of steps k to evenly divide the space from ymin to ymax
        numpoints = 7
        interval = (ymax-ymin)//numpoints
        stopmoving = False
        converge_limit = 100
        i=0
        for y_base in xrange(ymin,ymax,interval):
            #y_base = 150 #20 # this represents 3" in front of the car
            index_y = height - y_base 
            index_x = previous_x
            x_left, x_right,y_left,y_right = self.find_offset_in_lane(edges, index_x, index_y, width)  
            if (not(y_base==ymin) and (x_right-x_left)<converge_limit):
                x_left = leftpts[-1][0]
                x_right = rightpts[-1][0]
                y_left = leftpts[-1][1]
                y_right = rightpts[-1][1]
                stopmoving = True
            if stopmoving:
                x_left = leftpts[-1][0]
                x_right = rightpts[-1][0]
                y_left = leftpts[-1][1]
                y_right = rightpts[-1][1]
            midpointx = (x_right + x_left)//2
            midpointy = (y_right + y_left)//2
            midpts = np.array([(midpointx,midpointy)],dtype = np.int32)
            leftpts = np.array([(x_left,y_left)],dtype = np.int32)
            rightpts = np.array([(x_right,y_right)],dtype = np.int32)
            if (y_base==ymin):
                midpointlist = midpts
                leftlist = leftpts
                rightlist = rightpts
            else:
                midpointlist = np.concatenate((midpointlist,midpts))
                leftlist = np.concatenate((leftlist,leftpts))
                rightlist = np.concatenate((rightlist,rightpts))
            if (not(y_base==ymin)): 
                cv2.line(img, (midpointx, midpointy),(previous_x, previous_y), (0,255,255),3)
            cv2.circle(img, (x_right,y_right), 4, (0,255,255), -1)
            cv2.circle(img, (x_left,  y_left), 4, (0,255,255), -1)
            cv2.circle(img, (midpointx,  midpointy), 3, (0,255,255), -1)
            previous_x = midpointx
            previous_y = midpointy
            if self.statepoints:
                if (self.statepoints[1][i]>479):
                    self.statepoints[1][i] = 479
                if ((i>0) and self.count > 10):
                    previous_statex = self.statepoints[0][i-1]
                    previous_statey = self.statepoints[1][i-1]
                    cv2.line(img, (self.statepoints[0][i], self.statepoints[1][i]),(previous_statex, previous_statey), (0, 255,0),3)
                cv2.circle(img, (self.statepoints[0][i],self.statepoints[1][i]), 4, (0, 255,0), -1)
            if endtrack:
                break
            i+=1

        return midpointlist,leftlist,rightlist 

    def publish_states(self,midpointlist,leftlist,rightlist):
        if ((self.count == 5) and self.printme): 
            print("\nReference Trajectory")
            print(midpointlist)
        midpointlist[:,1] = 480-midpointlist[:,1]
        midlistx,midlisty = self.convertPixelsToDistance(midpointlist)
        self.reference_trajectory.x = midlistx.tolist()
        self.reference_trajectory.y = midlisty.tolist()
        #print(midlistx)
        if (midlisty[-1] <1.5*.3048):
            self.moving_pub.publish(False)
        else:
            self.moving_pub.publish(True)
        midlistpsi = np.empty(len(midlisty))
        self.reference_trajectory.psi = midlistpsi.tolist()
        if ((self.count == 5) and self.printme):
            print(self.reference_trajectory)
            
        self.reference_trajectory_pub.publish(self.reference_trajectory)

        ####################################################################
        # Uncomment this next line when you are ready to use LQR
        #self.compute_uOpt(self.reference_trajectory.x,self.reference_trajectory.y,self.v_ref)
        ####################################################################
    
    def toFloat(self,x):
        try:
            return float(x)
        except:
            return x

    def convertPixelsToDistance(self,inputarray):
        xlist = inputarray[:,0]
        ylist = inputarray[:,1]
        transformedxfootlist = np.float32(xlist)
        transformedyfootlist = np.float32(ylist)
        
        for i in np.arange(len(xlist)):
            x = xlist[i]
            y = ylist[i]
           
            transformedxfootlist[i] = self.calc_x(x,y)#0.3048*(x-320)/(-1.5*y+342) #number of xpixels from center divided by xpixels per foot
            transformedyfootlist[i] = self.calc_y(y)+0.15 # +0.15 for half of wheelbase

        return transformedxfootlist,transformedyfootlist

    def calc_x(self,x,y):
        #(x-320) is for xpixel to xnewpixel frame
        xnew = (x-320)/(FILL IN HERE)
        xnew=xnew*0.3048 #ft to m
        return xnew

    def calc_y(self,y):
        # y is already swapped into ynewpixel frame
        ynew = FILL IN HERE
        ynew=ynew*0.3048 #ft to m
        return ynew
    
    def compute_uOpt(self,x_refswapped,y_refswapped,v_ref):
        try:
            lr = 0.15
            lf = 0.15
            
            # changing the xnewpixel,ynewpixel frame to x_I,y_I inertial frame
            x_ref = [y_refswapped[0],y_refswapped[2]]
            y_ref = [-x_refswapped[0],-x_refswapped[2]]


            dt = self.dt

            #Assuming that the car is on the path, first point is at COM
            x_ref_for_radius = np.append([-lf],x_ref)
            y_ref_for_radius = np.append([0],y_ref);

            i = 0
            x1 = x_ref_for_radius[i];
            x2 = x_ref_for_radius[i+1];
            x3 = x_ref_for_radius[i+2];
            y1 = y_ref_for_radius[i];
            y2 = y_ref_for_radius[i+1];
            y3 = y_ref_for_radius[i+2];
            ma = (y2-y1)/(x2-x1);
            mb = (y3-y2)/(x3-x2);
            if ma == mb:
                y2 = y2 + 1e-5;
                ma = (y2-y1)/(x2-x1);
                mb = (y3-y2)/(x3-x2);

            x_c = (ma*mb*(y1-y3)+mb*(x1+x2)-ma*(x2+x3))/(2*(mb-ma));
            y_c = (-1/ma)*(x_c-(x1+x2)/2)+(y1+y2)/2;
            Radius = sqrt(pow((x2-x_c),2)+pow((y2-y_c),2));
            psidot_des = v_ref/Radius;
            psi_des = psidot_des*dt;

            if y_c>0:
                beta_des = abs(asin(lr/Radius))*-1;
            else:
                beta_des = abs(asin(lr/Radius));
            
            ##################################################
            #IMPLEMENT LQR HERE
            ##################################################

            self.uOpt_pub.publish(vOpt,deltaOpt)
        except :
            pass

def shutdown_func():
    cv2.destroyAllWindows()

def main(args):
    rospy.on_shutdown(shutdown_func)
    global image_processor_global
    global offset_global

    # Intialize the node
    rospy.init_node('image_processing_node', anonymous=True)

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
