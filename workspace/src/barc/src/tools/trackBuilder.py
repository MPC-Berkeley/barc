"""
Check track design before launching
This file was created by Shuqi Xu (shuqixu@berkeley.edu)
"""

from numpy import hstack, arange, array, transpose, pi, sin, cos
from numpy import size,  argmin, ceil
from numpy import ones, sqrt
import matplotlib.pyplot as plt

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
            d_theta = angle / length
            
            self.theta      = hstack((self.theta,self.theta[-1] + d_theta))
            self.curvature  = hstack((self.curvature,d_theta/ds))

    def createRaceTrack(self, name):
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
        if name == "race":
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
        
        elif name == "3110":
            # track for room 3110
            num = 80 # 60
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
            track_data=[[int(ceil(2*60)) ,0],
                        [int(ceil(2*num)), pi/2],
                        [int(ceil(2*(80+67))) ,0],
                        [int(ceil(2*200)), pi],
                        [int(ceil(2*20)) ,0],
                        [int(ceil(2*num)), -pi/2],
                        [int(ceil(2*44))  , 0],
                        [int(ceil(2*200)), pi],
                        [int(ceil(2*(110))) ,0]]  

        elif name == "basic":
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

        elif name == "rectangle":
            track_data = [[int(ceil(2.8*40)), 0],
                          [int(ceil(2.8*120)), -pi/2],
                          [int(ceil(2.8*5)), 0],
                          [int(ceil(2.8*120)), -pi/2],
                          [int(ceil(2.8*80)), 0],
                          [int(ceil(2.8*120)), -pi/2],
                          [int(ceil(2.8*5)), 0],
                          [int(ceil(2.8*120)), -pi/2],
                          [int(ceil(2.8*40)), 0]]

        elif name == "MSC_lab":
            # TRACK FOR MSC EXPERIMENT ROOM
            track_data = [[int(ceil(1.5*3*30)), 0],
                          [int(ceil(1.5*3*100)), pi],
                          [int(ceil(1.5*3*60)), 0],
                          [int(ceil(1.5*3*100)), pi],
                          [int(ceil(1.5*3*30)), 0]]
                      
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

if __name__ == '__main__':
    track = Track(0.01,0.8)
    track.createRaceTrack("3110")
    fig = plt.figure("Race",figsize=(12,10))
    ax  = fig.add_subplot(111)
    ax.plot(track.nodes[0,:],       track.nodes[1,:],       "k--", alpha=0.4)
    ax.plot(track.nodes_bound1[0,:],track.nodes_bound1[1,:],"r-")
    ax.plot(track.nodes_bound2[0,:],track.nodes_bound2[1,:],"r-")
    plt.show()
