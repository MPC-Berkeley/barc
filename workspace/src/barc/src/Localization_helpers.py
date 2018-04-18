"""This module includes all functions that are necessary to convert the current absolute position
(x, y, psi) and the given racetrack to the current relative position (s, eY, ePsi)"""
#from numpy import sin, cos, tan, array, dot, linspace, pi, ones, zeros, amin, argmin, arange
#from numpy import hstack, vstack, sign, size, diff, cumsum, mod, floor, interp, linalg
#from numpy import polyval, polyder, arctan2
from numpy import hstack, arange, array, zeros, vstack, transpose, pi, linspace, sin, cos, inf
from numpy import size, tan, diff, cumsum, mod, argmin, interp, floor, arctan2, polyval, polyder
from numpy import amin, dot, ones, sqrt, linalg, sign, sum
#from numpy import *
import scipy.optimize

def create_circle(rad, n, c):
    """This function creates a simple circular racetrack"""
    ang = linspace(0,2*pi,n)
    x = rad*cos(ang)+c[0]
    y = rad*sin(ang)+c[1]
    return array([x,y])

def add_curve(theta, curvature, length, angle):
    d_theta = 0
    ds = 0.03
    curve = 2*sum(arange(1,length/2+1,1))+length/2
    for i in range(0,length):
        if i < length/2+1:
            d_theta = d_theta + angle / curve
        else:
            d_theta = d_theta - angle / curve
        theta = hstack((theta,theta[-1] + d_theta))
        curvature = hstack((curvature,d_theta/ds))
    return theta, curvature

def create_bezier(p0,p1,p2,dt):
    t = arange(0,1+dt,dt)[:,None]
    p = (1-t)**2*p0 + 2*(1-t)*t*p1+t**2*p2
    return p

class Localization(object):
    # POSITION RELATED VARIABLES
    n                   = 0                     # number of nodes
    pos                 = 0                     # current position
    psi                 = 0                     # current orientation
    curv_curr           = 0                     # current track curvature
    # TRACK RELATED VARIABLES
    nodes               = array([0])            # x-y coordinate of the track
    nodes_bound1        = array([0])
    nodes_bound2        = array([0])
    ds                  = 0.03                  # distance between nodes
    curvature           = 0
    theta               = array([0])            # orientation of the track
    # POSITIONS RELATED VARIABLES
    s = 0                   # distance from s_start to current closest node (idx_min)
    ey = 0                  # lateral distance to path
    epsi = 0                # error in psi (between current psi and trajectory tangent)
    v = 0                   # current velocity (not necessary for these calculations but for MPC)
    x = 0
    y = 0
    v_x = 0
    v_y = 0
    psiDot = 0

    def create_track(self):
        x = array([0])           # starting point
        y = array([0])
        ds = 0.03
        theta = array([0])
        curvature = array([0])

        theta, curvature = add_curve(theta,curvature,60,0)
        theta, curvature = add_curve(theta,curvature,80,-pi/2)
        theta, curvature = add_curve(theta,curvature,20,0)
        theta, curvature = add_curve(theta,curvature,80,-pi/2)
        theta, curvature = add_curve(theta,curvature,40,pi/10)
        theta, curvature = add_curve(theta,curvature,60,-pi/5)
        theta, curvature = add_curve(theta,curvature,40,pi/10)
        theta, curvature = add_curve(theta,curvature,80,-pi/2)
        theta, curvature = add_curve(theta,curvature,20,0)
        theta, curvature = add_curve(theta,curvature,80,-pi/2)
        theta, curvature = add_curve(theta,curvature,75,0)

        for i in range(0,size(theta)):
            x = hstack((x, x[-1] + cos(theta[i])*ds))
            y = hstack((y, y[-1] + sin(theta[i])*ds))

        self.nodes = array([x,y])
        self.curvature = curvature
        self.ds = ds
        self.n = size(x)
        print "number of nodes: %i"%self.n
        print "length : %f"%((self.n)*ds)

    def create_feature_track(self):
        # BASIC PARAMETERS INITIALIZATION
        ds = 0.03
        width =0.8
        theta = array([pi/4])
        # X-Y TRACK/BOUND COORDINATE INITILIZATION 
        x = array([0])
        bound1_x = array([0])
        bound2_x = array([0])
        y = array([0])
        bound1_y = array([0])
        bound2_y = array([0])
        bound1_x = x + width/2*cos(theta[0]+pi/2)
        bound2_x = x - width/2*cos(theta[0]+pi/2)
        bound1_y = y + width/2*sin(theta[0]+pi/2)
        bound2_y = y - width/2*sin(theta[0]+pi/2)
        # CURVATURE ARRAY INITIALIZATION
        curvature = array([0])
        # TRACK DESIGN CALCULATIONS
        v = 2.5
        max_a=7.6;
        R=v**2/max_a
        max_c=1/R
        angle=(pi+pi/2)-0.105
        R_kin = 0.8
        num_kin = int(round(angle/ ( 0.03/R_kin ) * 2))
        num = max(int(round(angle/ ( 0.03/R ) * 2)),num_kin)
        # TRACK DATA CALCULATION
        theta, curvature = add_curve(theta,curvature,num,-angle)
        theta, curvature = add_curve(theta,curvature,num,angle)
        for i in range(1,size(theta)):
            x_next = x[-1] + cos(theta[i])*ds
            y_next = y[-1] + sin(theta[i])*ds
            bound1_x_next = x_next + width/2*cos(theta[i]+pi/2)
            bound2_x_next = x_next - width/2*cos(theta[i]+pi/2)
            bound1_y_next = y_next + width/2*sin(theta[i]+pi/2)
            bound2_y_next = y_next - width/2*sin(theta[i]+pi/2)
            x = hstack((x, x_next))
            y = hstack((y, y_next))
            bound1_x = hstack((bound1_x, x_next+ width/2*cos(theta[i]+pi/2)))
            bound2_x = hstack((bound2_x, x_next- width/2*cos(theta[i]+pi/2)))
            bound1_y = hstack((bound1_y, y_next+ width/2*sin(theta[i]+pi/2)))
            bound2_y = hstack((bound2_y, y_next- width/2*sin(theta[i]+pi/2)))
        # LOCALIZATION OBJECT DATA WRITING
        self.nodes = array([x,y])
        self.nodes_bound1 = array([bound1_x,bound1_y])
        self.nodes_bound2 = array([bound2_x,bound2_y])
        self.curvature = curvature
        self.ds = ds
        self.n = size(x)
        print "number of nodes: %i"%self.n
        print "length : %f"%((self.n)*ds)

    def set_pos(self,x,y,psi,v_x,v_y,psiDot):
        self.pos = array([x,y])
        self.psi = psi
        self.v = sqrt(v_x**2+v_y**2)
        self.x = x
        self.y = y
        self.v_x = v_x
        self.v_y = v_y
        self.psiDot = psiDot

    def find_s(self):
        dist        = sum((self.pos*ones([self.n,2])-self.nodes.transpose())**2,1)**0.5 # distance of current position to all nodes
        idx_min     = argmin(dist)              # index of minimum distance
        self.s      = 0.03*idx_min

        dist_bound1 = (self.pos[0]-self.nodes_bound1[0,idx_min])**2 + (self.pos[1]-self.nodes_bound1[1,idx_min])**2
        dist_bound2 = (self.pos[0]-self.nodes_bound2[0,idx_min])**2 + (self.pos[1]-self.nodes_bound2[1,idx_min])**2
        
        ey = dist[idx_min]
        if dist_bound1>dist_bound2:
            self.ey = ey
        else:
            self.ey = -ey

        self.epsi      = self.psi - self.theta[idx_min]
        self.curv_curr = self.curvature[idx_min]

    def __init__(self):
        self.x = 0
        self.y = 0
