"""
This module includes all functions that are necessary to convert the current absolute position
(x, y, psi) and the given racetrack to the current relative position (s, eY, ePsi)
This file was created by Shuqi Xu (shuqixu@berkeley.edu)
"""

from numpy import hstack, arange, array, transpose, pi, sin, cos
from numpy import size,  argmin, ceil
from numpy import ones, sqrt, sum

class Track(object):
    """ Object collecting  estimated state data
    Attributes:
        addCurve():
            Helper function for track construction
        create_feature_track():
            Create track to collect feature data
        create_race_track():
            Create track to do racing
        Localize():
            Localize the position on the current map
    """
    def __init__(self,ds,width):
        
        # Track construction
        self.nodes          = None         # track x-y array 
        self.nodes_bound1   = None         # track bound x-y array
        self.nodes_bound2   = None         # track bound x-y array
        self.track_s        = None         # track s array
        self.track_idx      = None         # track idx array
        self.curvature      = None         # track curvature array
        self.theta          = None         # track tangent angle array
        
        # Track parameters
        self.ds    = ds      # discretization distance
        self.width = width   # track width
        self.n     = 0       # number of nodes

        # POSITIONS RELATED VARIABLES
        self.s      = 0.
        self.ey     = 0.
        self.epsi   = 0.
        self.x      = 0.      
        self.y      = 0.

        self.pos        = [0.,0.]      # current position
        self.psi        = 0.           # current orientation
        self.curv_curr  = 0.           # current track curvature
        self.idx_curr   = 0            # current track position


    def addCurve(self, length, angle):
        """
        Arguments:
            theta: track tangent angle array, which has been constructed
            curvature: track curvature array, which has been constructed
            length: next section number of points
            angle: next section radian
        """
        d_theta = 0
        ds = self.ds
        
        curve = 2*sum(arange(1,length/2+1,1))+length/2
        
        for i in range(0,length):
            if i < length/2+1:
                d_theta = d_theta + angle / curve
            else:
                d_theta = d_theta - angle / curve
            
            self.theta      = hstack((self.theta,self.theta[-1] + d_theta))
            self.curvature  = hstack((self.curvature,d_theta/ds))

    def createFeatureTrack(self):
        """Track construction for feature data collecting"""

        # TRACK PARAMETERS INITIALIZATION
        ds      = self.ds
        width   = self.width
        self.theta      = array([pi/4])
        self.curvature  = array([0])
        
        # X-Y TRACK/BOUND COORDINATE INITILIZATION 
        x = array([0])
        bound1_x = array([0])
        bound2_x = array([0])
        y = array([0])
        bound1_y = array([0])
        bound2_y = array([0])
        bound1_x = x + width/2*cos(self.theta[0]+pi/2)
        bound2_x = x - width/2*cos(self.theta[0]+pi/2)
        bound1_y = y + width/2*sin(self.theta[0]+pi/2)
        bound2_y = y - width/2*sin(self.theta[0]+pi/2)
        
        # TRACK DESIGN
        v       = 2.5
        max_a   = 7.6;
        R       = v**2/max_a
        max_c   = 1/R
        angle   = (pi+pi/2)-0.105
        R_kin   = 0.8
        num_kin = int(round(angle/ ( ds/R_kin ) * 2))
        num     = max(int(round(angle/ ( ds/R ) * 2)),num_kin)
        num     = int(ceil(num*1.0))
        
        # TRACK CONSTRUCTION
        self.addCurve(num,-angle)
        self.addCurve(num,angle)
        
        for i in range(1,size(self.theta)):
            x_next = x[-1] + cos(self.theta[i])*ds
            y_next = y[-1] + sin(self.theta[i])*ds
            bound1_x_next = x_next + width/2*cos(self.theta[i]+pi/2)
            bound2_x_next = x_next - width/2*cos(self.theta[i]+pi/2)
            bound1_y_next = y_next + width/2*sin(self.theta[i]+pi/2)
            bound2_y_next = y_next - width/2*sin(self.theta[i]+pi/2)
            x = hstack((x, x_next))
            y = hstack((y, y_next))
            bound1_x = hstack((bound1_x, x_next+ width/2*cos(self.theta[i]+pi/2)))
            bound2_x = hstack((bound2_x, x_next- width/2*cos(self.theta[i]+pi/2)))
            bound1_y = hstack((bound1_y, y_next+ width/2*sin(self.theta[i]+pi/2)))
            bound2_y = hstack((bound2_y, y_next- width/2*sin(self.theta[i]+pi/2)))
        
        self.nodes          = array([x,y])
        self.nodes_bound1   = array([bound1_x,bound1_y])
        self.nodes_bound2   = array([bound2_x,bound2_y])
        self.n = size(x)
        self.track_s    = array([ds*i for i in range(self.n)])
        self.track_idx  = array([i for i in range(self.n)])

        print "number of nodes: %i"%self.n, "length: %f"%((self.n)*ds)

    def createRaceTrack(self):
        """Track construction for feature data collecting"""

        # TRACK PARAMETERS INITIALIZATION
        ds      = self.ds
        width   = self.width
        self.theta      = array([0])
        self.curvature  = array([0])
        
        # X-Y TRACK/BOUND COORDINATE INITILIZATION 
        x = array([0])
        bound1_x = array([0])
        bound2_x = array([0])
        y = array([0])
        bound1_y = array([0])
        bound2_y = array([0])
        bound1_x = x + width/2*cos(self.theta[0]+pi/2)
        bound2_x = x - width/2*cos(self.theta[0]+pi/2)
        bound1_y = y + width/2*sin(self.theta[0]+pi/2)
        bound2_y = y - width/2*sin(self.theta[0]+pi/2)

        # TRACK DATA CALCULATION
        # track_data=[[80,0],
        #             [120,-pi/2],
        #             [80,0],
        #             [220,-pi*0.85],
        #             [105,pi/15],
        #             [300,pi*1.15],
        #             [240,-pi*0.865],
        #             [100,0],
        #             [120,-pi/2],
        #             [153,0],
        #             [120,-pi/2],
        #             [211,0]]
        
        # track for room 3110
        # num = 100 # 60
        # track_data=[[int(ceil(2*80)) ,0],
        #             [int(ceil(2*num)), pi/2],
        #             [int(ceil(2*(80+47))) ,0],
        #             [int(ceil(2*num)), pi/2],
        #             [int(ceil(2*50)) ,0],
        #             [int(ceil(2*num)), pi/2],
        #             [int(ceil(2*4))  , 0],
        #             [int(ceil(2*num)), -pi/2],
        #             [int(ceil(2*30)) ,0],
        #             [int(ceil(2*num)), pi/2],
        #             [int(ceil(2*4)) ,0],
        #             [int(ceil(2*num)), pi/2],
        #             [int(ceil(2*(71+48))) ,0]]  

        # Basic track for experiment          
        track_data = [[int(ceil(3*60)), 0],
                      [int(ceil(3*80)), pi/2],
                      [int(ceil(3*20)), 0],
                      [int(ceil(3*80)), pi/2],
                      [int(ceil(3*40)), -pi/10],
                      [int(ceil(3*60)), pi/5],
                      [int(ceil(3*40)), -pi/10],
                      [int(ceil(3*80)), pi/2],
                      [int(ceil(3*20)), 0],
                      [int(ceil(3*80)), pi/2],
                      [int(ceil(3*75)), 0]]

        # track_data = [[int(ceil(2.8*40)), 0],
        #               [int(ceil(2.8*120)), -pi/2],
        #               [int(ceil(2.8*5)), 0],
        #               [int(ceil(2.8*120)), -pi/2],
        #               [int(ceil(2.8*80)), 0],
        #               [int(ceil(2.8*120)), -pi/2],
        #               [int(ceil(2.8*5)), 0],
        #               [int(ceil(2.8*120)), -pi/2],
        #               [int(ceil(2.8*40)), 0]]
        
        # TRACK FOR MSC EXPERIMENT ROOM
        # track_data = [[int(ceil(1.5*3*10)), 0],
        #               [int(ceil(1.5*3*120)), pi],
        #               [int(ceil(1.5*3*20)), 0],
        #               [int(ceil(1.5*3*120)), pi],
        #               [int(ceil(1.5*3*10)), 0]]
                      
        #   TRACK CONSTRUCTION
        for i in range(len(track_data)):
            num     = track_data[i][0]
            angle   = track_data[i][1]
            self.addCurve(num,angle)
            
        for i in range(1,size(self.theta)):
            x_next = x[-1] + cos(self.theta[i])*ds
            y_next = y[-1] + sin(self.theta[i])*ds
            bound1_x_next = x_next + width/2*cos(self.theta[i]+pi/2)
            bound2_x_next = x_next - width/2*cos(self.theta[i]+pi/2)
            bound1_y_next = y_next + width/2*sin(self.theta[i]+pi/2)
            bound2_y_next = y_next - width/2*sin(self.theta[i]+pi/2)
            x = hstack((x, x_next))
            y = hstack((y, y_next))
            bound1_x = hstack((bound1_x, x_next+ width/2*cos(self.theta[i]+pi/2)))
            bound2_x = hstack((bound2_x, x_next- width/2*cos(self.theta[i]+pi/2)))
            bound1_y = hstack((bound1_y, y_next+ width/2*sin(self.theta[i]+pi/2)))
            bound2_y = hstack((bound2_y, y_next- width/2*sin(self.theta[i]+pi/2)))
        
        self.nodes          = array([x,y])
        self.nodes_bound1   = array([bound1_x,bound1_y])
        self.nodes_bound2   = array([bound2_x,bound2_y])
        self.n = size(x)
        self.track_s    = array([ds*i for i in range(self.n)])
        self.track_idx  = array([i for i in range(self.n)])

        print "Python: %f m"%((self.n-1)*ds), ", %i"%self.n

    def Localize(self,x,y,psi):
        """ Localization on the constructed track
        Arguements:
            1. x    (global x-y)
            2. y    (global x-y)
            3. psi  (global x-y)
        """

        self.pos = array([x,y])
        self.psi = psi
        self.x = x
        self.y = y

        # 1. Localize s
        l_ran = 1 # LOCALIZATION RANGE: useful when track is overlaping
        # if self.s > self.track_s[-1]-l_ran:
        #     idx_candidate_1     = (self.track_s+l_ran>=self.track_s[-1])
        #     idx_candidate_2     = (self.track_s<=l_ran)
        #     x_candidate         = hstack((self.nodes[0,:][idx_candidate_1],self.nodes[0,:][idx_candidate_2]))
        #     y_candidate         = hstack((self.nodes[1,:][idx_candidate_1],self.nodes[1,:][idx_candidate_2]))
        #     nodes_candidate     = array([x_candidate,y_candidate])
        #     s_candidate         = hstack((self.track_s[idx_candidate_1],self.track_s[idx_candidate_2]))
        #     idx_curr_candidate  = hstack((self.track_idx[idx_candidate_1],self.track_idx[idx_candidate_2]))
        # else:


        dist_s              = self.track_s - self.s
        # idx_candidate       = (dist_s>=0) & (dist_s<l_ran) # the car can travel 0.3m maximumly within 0.1s  
        idx_candidate       = dist_s>-10000
        x_candidate         = self.nodes[0,:][idx_candidate]
        y_candidate         = self.nodes[1,:][idx_candidate]
        nodes_candidate     = array([x_candidate,y_candidate])
        s_candidate         = self.track_s[idx_candidate]
        idx_curr_candidate  = self.track_idx[idx_candidate]


        

        dist            = sum((self.pos*ones([len(x_candidate),2])-nodes_candidate.transpose())**2,1)
        idx_min         = argmin(dist)
        self.s          = s_candidate[idx_min]
        self.idx_curr   = idx_curr_candidate[idx_min]

        dist_bound1 = (self.pos[0]-self.nodes_bound1[0,self.idx_curr])**2 + (self.pos[1]-self.nodes_bound1[1,self.idx_curr])**2
        dist_bound2 = (self.pos[0]-self.nodes_bound2[0,self.idx_curr])**2 + (self.pos[1]-self.nodes_bound2[1,self.idx_curr])**2
        
        # 2. Localize ey
        ey = dist[idx_min]**0.5
        if dist_bound1>dist_bound2:
            self.ey = -ey
        else:
            self.ey = ey

        # 3. Localize epsi
        self.epsi      = (self.psi - self.theta[self.idx_curr]+pi)%(2*pi)-pi

        # 4. Localize curvature
        self.curv_curr = self.curvature[self.idx_curr]

        return self.s, self.ey, self.epsi
