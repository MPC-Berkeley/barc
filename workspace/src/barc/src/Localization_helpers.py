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
    idx_curr            = 0
    # TRACK RELATED VARIABLES
    nodes               = array([0])            # x-y coordinate of the track
    nodes_bound1        = array([0])
    nodes_bound2        = array([0])
    ds                  = 0.03                  # distance between nodes
    track_s             = 0
    track_idx           = 0
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
        self.track_s = array([ds*0.03 for i in range(self.n)])
        print "number of nodes: %i"%self.n
        print "length : %f"%((self.n)*ds)

    def create_feature_track(self):
        # BASIC PARAMETERS INITIALIZATION
        ds = 0.03
        width =0.8
        theta = array([pi/4])
        # theta = array([0])
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
        # num*=2
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
        self.theta = theta
        self.curvature = curvature
        self.n = size(x)
        self.track_s = array([0.03*i for i in range(self.n)])
        self.track_idx = array([i for i in range(self.n)])
        self.ds = ds
        print "number of nodes: %i"%self.n
        print "length : %f"%((self.n)*ds)

    def create_race_track(self):
        # BASIC PARAMETERS INITIALIZATION
        ds = 0.03
        width =0.8
        theta = array([0])
        # theta = array([0])
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

        # TRACK DATA CALCULATION
        track_data=[[80,0],
                    [120,-pi/2],
                    [80,0],
                    [220,-pi*0.85],
                    [105,pi/15],
                    [300,pi*1.15],
                    [240,-pi*0.865],
                    [100,0],
                    [120,-pi/2],
                    [153,0],
                    [120,-pi/2],
                    [211,0]]
        for i in range(len(track_data)):
            num = track_data[i][0]
            angle = track_data[i][1]
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
        self.theta = theta
        self.curvature = curvature
        self.n = size(x)
        self.track_s = array([0.03*i for i in range(self.n)])
        self.track_idx = array([i for i in range(self.n)])
        self.ds = ds
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
        if self.s > self.track_s[-1]-0.3:
            idx_candidate_1 = (self.track_s+0.3>=self.track_s[-1])
            idx_candidate_2 = (self.track_s<=0.3)
            x_candidate = hstack((self.nodes[0,:][idx_candidate_1],self.nodes[0,:][idx_candidate_2]))
            y_candidate = hstack((self.nodes[1,:][idx_candidate_1],self.nodes[1,:][idx_candidate_2]))
            nodes_candidate = array([x_candidate,y_candidate])
            s_candidate = hstack((self.track_s[idx_candidate_1],self.track_s[idx_candidate_2]))
            idx_curr_candidate = hstack((self.track_idx[idx_candidate_1],self.track_idx[idx_candidate_2]))
            
        else:
            dist_s = self.track_s - self.s
            idx_candidate = (dist_s>=0) & (dist_s<0.3) # the car can travel 0.3m maximumly within 0.1s  
            x_candidate = self.nodes[0,:][idx_candidate]
            y_candidate = self.nodes[1,:][idx_candidate]
            nodes_candidate = array([x_candidate,y_candidate])
            s_candidate = self.track_s[idx_candidate]
            idx_curr_candidate = self.track_idx[idx_candidate]

        dist        = sum((self.pos*ones([len(x_candidate),2])-nodes_candidate.transpose())**2,1) # distance of current position to all nodes, sqrt() is not necessary
        idx_min     = argmin(dist)              # index of minimum distance
        self.s      = s_candidate[idx_min]
        self.idx_curr = idx_curr_candidate[idx_min]

        dist_bound1 = (self.pos[0]-self.nodes_bound1[0,self.idx_curr])**2 + (self.pos[1]-self.nodes_bound1[1,self.idx_curr])**2
        dist_bound2 = (self.pos[0]-self.nodes_bound2[0,self.idx_curr])**2 + (self.pos[1]-self.nodes_bound2[1,self.idx_curr])**2
        ey = dist[idx_min]**0.5
        if dist_bound1>dist_bound2:
            self.ey = -ey
        else:
            self.ey = ey
        # print("length of self.theta is "+repr(len(self.theta)))
        self.epsi      = (self.psi - self.theta[self.idx_curr]+pi)%(2*pi)-pi
        self.curv_curr = self.curvature[self.idx_curr]

    def __init__(self):
        self.x = 0
        self.y = 0
