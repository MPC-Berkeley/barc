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

def add_curve(theta, length, angle):
    d_theta = 0
    curve = 2*sum(arange(1,length/2+1,1))+length/2
    for i in range(0,length):
        if i < length/2+1:
            d_theta = d_theta + angle / curve
        else:
            d_theta = d_theta - angle / curve
        theta = hstack((theta,theta[-1] + d_theta))
    return theta

def create_bezier(p0,p1,p2,dt):
    t = arange(0,1+dt,dt)[:,None]
    p = (1-t)**2*p0 + 2*(1-t)*t*p1+t**2*p2
    return p

class Localization(object):
    """This is the main class which includes all variables and functions to calculate the current relative position."""
    n                   = 0                     # number of nodes
    c                   = 0                     # center of circle (in case of circular trajectory)
    pos                 = 0                     # current position
    psi                 = 0                     # current orientation
    nodes               = array([0])            # all nodes are saved in a matrix
    N_nodes_poly_back   = 20                    # number of nodes behind current position
    N_nodes_poly_front  = 100                    # number of nodes in front
    ds                  = 0                     # distance between nodes
    nPoints             = N_nodes_poly_front+N_nodes_poly_back+1    # number of points for interpolation in total
    OrderXY             = 10                     # order of x-y-polynomial interpolation
    OrderThetaCurv      = 8                     # order of theta interpolation
    closed              = True                  # open or closed trajectory?

    coeffCurvature = zeros(OrderThetaCurv + 1)

    s = 0                   # distance from s_start to current closest node (idx_min)
    s_start = 0             # distance along path from first node to start node (which is N_nodes_poly_back behind current closest node)
    ey = 0                  # lateral distance to path
    epsi = 0                # error in psi (between current psi and trajectory tangent)
    v = 0                   # current velocity (not necessary for these calculations but for MPC)
    x = 0
    y = 0
    v_x = 0
    v_y = 0
    psiDot = 0

    def create_circle(self,rad=1.0,n=100,c=array([0,0])):           # note: need to make sure that all points are equidistant!
        ang = linspace(0,2*pi-2*pi/n,n)
        #print(ang)
        x = rad*cos(ang)+c[0]
        y = rad*sin(ang)+c[1]
        self.nodes  = array([x,y])
        self.n      = n
        self.c      = c
        self.rad    = rad
        #self.ds    = rad*2*pi/n
        self.ds     = 2*rad*tan(2*pi/n/2)

    def create_track2(self):
        p0 = array([[0,0],
            [2,0],
            [2,-2],
            [1,-4],
            [3,-4],
            [5,-5],
            [3,-6],
            [1,-6],
            [-0.5,-5.5],
            [-1.5,-5],
            [-3,-4],
            [-3,-2],
            [0,0]])
        p1 = array([0,0,0.75,-1,1,inf,0,0,-1,0,inf,inf,0])

        p = p0[0,:]
        for i in range(0,size(p0,0)-1):
            b1 = p0[i,1] - p1[i]*p0[i,0]
            b2 = p0[i+1,1] - p1[i+1]*p0[i+1,0]
            a1 = p1[i]
            a2 = p1[i+1]
            x = (b2-b1)/(a1-a2)
            y = a1*x + b1
            if p1[i] == inf:
                x = p0[i,0]
                y = a2*x+b2
            elif p1[i+1] == inf:
                x = p0[i+1,0]
                y = a1*x+b1
            if a1 == a2:
                p = vstack((p,p0[i+1,:]))
            else:
                p = vstack((p,create_bezier(p0[i,:],array([[x,y]]),p0[i+1,:],0.01)))
        self.nodes = transpose(p)
        self.ds = 0.06

    def create_track(self):
        x = array([0])           # starting point
        y = array([0])
        ds = 0.03
        theta = array([0])

        # Sophisticated racetrack: length = 25.62m
        # theta = add_curve(theta,30,0)
        # theta = add_curve(theta,60,-2*pi/3)
        # #theta = add_curve(theta,30,0)
        # theta = add_curve(theta,90,pi)
        # theta = add_curve(theta,80,-5*pi/6)
        # theta = add_curve(theta,10,0)
        # theta = add_curve(theta,50,-pi/2)
        # theta = add_curve(theta,50,0)
        # theta = add_curve(theta,40,-pi/4)
        # theta = add_curve(theta,30,pi/4)
        # theta = add_curve(theta,20,0)
        # theta = add_curve(theta,50,-pi/2)
        # theta = add_curve(theta,25,0)
        # theta = add_curve(theta,50,-pi/2)
        # theta = add_curve(theta,28,0)

        # # SIMPLE RACETRACK (smooth curves): length = 24.0m
        # theta = add_curve(theta,10,0)
        # theta = add_curve(theta,80,-pi)
        # theta = add_curve(theta,20,0)
        # theta = add_curve(theta,80,-pi)
        # theta = add_curve(theta,9,0)

        # AGGRESSIVE GOGGLE TRACK: length = 17.76m
        # theta = add_curve(theta,30,0)
        # theta = add_curve(theta,40,-pi/2)
        # theta = add_curve(theta,40,-pi/2)
        # theta = add_curve(theta,20,-pi/6)
        # theta = add_curve(theta,30,pi/3)
        # theta = add_curve(theta,20,-pi/6)
        # theta = add_curve(theta,40,-pi/2)
        # theta = add_curve(theta,40,-pi/2)
        # theta = add_curve(theta,35,0)

        # SIMPLE GOGGLE TRACK: length = 17.94m
        # theta = add_curve(theta,30,0)
        # theta = add_curve(theta,40,-pi/2)
        # #theta = add_curve(theta,10,0)
        # theta = add_curve(theta,40,-pi/2)
        # theta = add_curve(theta,20,pi/10)
        # theta = add_curve(theta,30,-pi/5)
        # theta = add_curve(theta,20,pi/10)
        # theta = add_curve(theta,40,-pi/2)
        # #theta = add_curve(theta,10,0)
        # theta = add_curve(theta,40,-pi/2)
        # theta = add_curve(theta,37,0)

        # GOGGLE TRACK WITH STRAIGHT LINES, LENGTH = 19.11m (using ds = 0.03m)

        theta = add_curve(theta,60,0)
        theta = add_curve(theta,80,-pi/2)
        theta = add_curve(theta,20,0)
        theta = add_curve(theta,80,-pi/2)
        theta = add_curve(theta,40,pi/10)
        theta = add_curve(theta,60,-pi/5)
        theta = add_curve(theta,40,pi/10)
        theta = add_curve(theta,80,-pi/2)
        theta = add_curve(theta,20,0)
        theta = add_curve(theta,80,-pi/2)
        theta = add_curve(theta,75,0)


        # SMALL GOOGLE FOR 3110

        # theta = add_curve(theta,50,0)
        # theta = add_curve(theta,80,-pi/2)
        # theta = add_curve(theta,20,0)
        # theta = add_curve(theta,80,-pi/2)
        # theta = add_curve(theta,35,-pi/10)
        # theta = add_curve(theta,50,pi/5)
        # theta = add_curve(theta,35,-pi/10)
        # theta = add_curve(theta,80,-pi/2)
        # theta = add_curve(theta,20,0)
        # theta = add_curve(theta,80,-pi/2)
        # theta = add_curve(theta,65,0)


        ######################################################
        # TEST TRACKS

        # theta = add_curve(theta,20,0)
        # theta = add_curve(theta,40,-pi/2)
        # theta = add_curve(theta,10,0)

        # theta = add_curve(theta,40,-pi/2)
        # theta = add_curve(theta,230,0)

        # theta = add_curve(theta,40,-pi/2)
        # theta = add_curve(theta,10,0)

        # theta = add_curve(theta,40,-pi/2)
        # theta = add_curve(theta,205,0)



        # theta = add_curve(theta,130,0)
        # theta = add_curve(theta,80,-pi/2)
        # theta = add_curve(theta,20,0)
        # theta = add_curve(theta,80,-pi/2)
        # theta = add_curve(theta,40,pi/10)
        # theta = add_curve(theta,60,-pi/5)
        # theta = add_curve(theta,40,pi/10)
        # theta = add_curve(theta,80,-pi/2)
        # theta = add_curve(theta,20,0)
        # theta = add_curve(theta,80,-pi/2)
        # theta = add_curve(theta,5,0)


        # theta = add_curve(theta,10,0)
        # theta = add_curve(theta,80,-pi/2)
        # theta = add_curve(theta,20,0)
        # theta = add_curve(theta,80,-pi/2)
        # theta = add_curve(theta,40,pi/10)
        # theta = add_curve(theta,60,-pi/5)
        # theta = add_curve(theta,40,pi/10)
        # theta = add_curve(theta,80,-pi/2)
        # theta = add_curve(theta,20,0)
        # theta = add_curve(theta,80,-pi/2)
        # theta = add_curve(theta,125,0)


        # theta = add_curve(theta,10,0)
        # theta = add_curve(theta,60,-pi/2)
        # theta = add_curve(theta,60,-pi/2)
        # theta = add_curve(theta,50,0)
        # theta = add_curve(theta,60,-pi/2)
        # theta = add_curve(theta,60,-pi/2)
        # theta = add_curve(theta,40,0)

        ############################################################


        # SHORT SIMPLE RACETRACK (smooth curves): 12.0m

        # theta = add_curve(theta,5,0)
        # theta = add_curve(theta,40,-pi)
        # theta = add_curve(theta,10,0)
        # theta = add_curve(theta,40,-pi)
        # theta = add_curve(theta,4,0)

        # SIMPLER RACETRACK (half circles as curves):

        for i in range(0,size(theta)):
            x = hstack((x, x[-1] + cos(theta[i])*ds))
            y = hstack((y, y[-1] + sin(theta[i])*ds))


        self.nodes = array([x,y])
        self.ds = ds
        self.n = size(x)
        print "number of nodes: %i"%self.n
        print "length : %f"%((self.n)*ds)

    def create_racetrack(self,L=1.0,b=1.0,ds=0.5,c=array([0,0]),ang=0):     # problem: points are not equidistant at connecing points
        x = linspace(0,L/2.0,5)#arange(0,L/2.0,ds)                                          # otherwise: would create a racetrack with parallel lines
        s = size(x)
        da = 2.0*pi/360*5
        x = hstack((x,L/2.0+b/2*cos(arange(pi/2,-pi/2,-da))))
        x = hstack((x,linspace(L/2.0,-L/2.0,5)))
        x = hstack((x,-L/2.0+b/2*cos(arange(pi/2,1.5*pi+da,da))))
        x = hstack((x,linspace(-L/2.0,0,5)))

        y = ones(s)*b/2.0
        y = hstack((y,b/2*sin(arange(pi/2,-pi/2,-da))))
        y = hstack((y,-ones(s)*b/2))
        y = hstack((y,-b/2*sin(arange(pi/2,1.5*pi+da,da))))
        y = hstack((y,ones(s)*b/2.0))

        s_tot = size(x)

        R = array([[cos(ang),-sin(ang)],[sin(ang),cos(ang)]])       # Rotation Matrix

        nodes = dot(R,array([x,y]))+(c*ones([2,s_tot]).T).T

        self.nodes = nodes
        self.n = size(x)
        self.ds = ds

    def create_ellipse(self,L_a,L_b,n,c=array([0,0])):
        ang = linspace(0,2*pi,n)
        x = L_a*cos(ang)+c[0]
        y = L_b*sin(ang)+c[1]
        self.nodes  = array([x,y])
        self.n      = n
        self.c      = c

    def prepare_trajectory(self,ds_in):     # brings trajectory to correct format (equidistant nodes with distances around ds)
        x = self.nodes[0]
        y = self.nodes[1]
        n = size(x)                             # number of nodes
        ds = (diff(x)**2+diff(y)**2)**0.5       # distances between nodes
        #if self.closed:
        #   ds = hstack((ds,((x[n-1]-x[0])**2+(y[n-1]-y[0])**2)**0.5))
        s = hstack((0,cumsum(ds)))
        length = sum(ds)
        dsn = ds_in                     # optimal new step size (might be calculated externally for now)
        rem = lambda x:mod(length,x)    # function to calculate optimal step size (for evenly distributed steps)
        sol = scipy.optimize.fmin(func=rem,x0=dsn,args=(),xtol=0.0000001,ftol=0.0000001)
        dsn = sol[0]
        #dsn = 0.06
        #sn = arange(0,length,dsn)       # new steps
        #n = size(sn,0)
        n = floor(length/dsn)
        #n = 500
        #print(n)
        sn = arange(0,n*dsn,dsn)
        #print(sn)
        xn = interp(sn,s,x)
        yn = interp(sn,s,y)

        self.nodes = array([xn,yn])
        self.ds = dsn
        self.n = size(xn)
        print "Finished track optimization."
        print "Track length = %fm, ds = %fm"%(length,dsn)
        print "Approximated length in region of s = [%.3f, %.3f]"%((self.N_nodes_poly_back-5)*dsn,(self.nPoints-5)*dsn)
        print "# Nodes: %f"%n
        #print(sn)
        #print(self.nodes)


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

        

        n           = self.n                    # number of nodes
        nPoints     = self.nPoints              # number of points for polynomial approximation (around current position)

        # Use closest node to determine start and end of polynomial approximation
        idx_start = idx_min - self.N_nodes_poly_back
        idx_end   = idx_min + self.N_nodes_poly_front

        n_poly = self.N_nodes_poly_back + self.N_nodes_poly_front + 1

        s_start = idx_start * self.ds

        if self.closed == True:                 # if the track is modeled as closed (start = end)
            if idx_start<0:                     # and if the start for polynomial approx. is before the start line
                nodes_X = hstack((self.nodes[0,n+idx_start:n],self.nodes[0,0:idx_end+1]))       # then stack the end and beginning of a lap together
                nodes_Y = hstack((self.nodes[1,n+idx_start:n],self.nodes[1,0:idx_end+1]))
                #nodes_X = hstack((linspace(idx_start,-1,-idx_start)*self.ds,self.nodes[0,0:idx_end+1]))
                #nodes_Y = hstack((zeros(-idx_start),self.nodes[1,0:idx_end+1]))

                idx_start = n+idx_start
            elif idx_end>n-1:                   # if the end is behind the finish line
                nodes_X = hstack((self.nodes[0,idx_start:n],self.nodes[0,0:idx_end+1-n]))       # then stack the end and beginning of the lap together
                nodes_Y = hstack((self.nodes[1,idx_start:n],self.nodes[1,0:idx_end+1-n]))
                #nodes_X = hstack((self.nodes[0,idx_start:n],self.nodes[0,n-1]+linspace(1,idx_end-n+1,idx_end-n+1)*self.ds))
                #nodes_Y = hstack((self.nodes[1,idx_start:n],ones(idx_end-n+1)*self.nodes[1,n-1]))
            else:                               # if we are somewhere in the middle of the track
                nodes_X = self.nodes[0,idx_start:idx_end+1]     # then just use the nodes from idx_start to end for interpolation
                nodes_Y = self.nodes[1,idx_start:idx_end+1]
        else:                                   # if the track is not closed
            if idx_start<0:
                nodes_X = self.nodes[0,0:nPoints]
                nodes_Y = self.nodes[1,0:nPoints]
                idx_start = 0
            elif idx_end>n-1:
                nodes_X = self.nodes[0,n-nPoints:n]
                nodes_Y = self.nodes[1,n-nPoints:n]
                idx_start = n-nPoints
            else:
                nodes_X = self.nodes[0,idx_start:idx_end+1]
                nodes_Y = self.nodes[1,idx_start:idx_end+1]

        # Create Matrix for interpolation
        # x-y-Matrix
        # The x-y-matrix is just filled with s^n values (n from 0 to the polynomial degree) for s values between s = 0 and s = nPoints*ds
        # with nPoints = number of points that approximate the polynomial
        Matrix = zeros([self.nPoints,self.OrderXY+1])
        for i in range(0,self.nPoints):
            for k in range(0,self.OrderXY+1):
                Matrix[i,self.OrderXY-k] = (s_start + i*self.ds)**k

        # curvature matrix
        Matrix3rd = zeros([self.nPoints,self.OrderThetaCurv+1])
        for i in range(0,self.nPoints):
            for k in range(0,self.OrderThetaCurv+1):
                Matrix3rd[i,self.OrderThetaCurv-k] = (s_start + i*self.ds)**k

        # Solve system of equations to find polynomial coefficients (for x-y)
        coeffX = linalg.lstsq(Matrix,nodes_X)[0]
        coeffY = linalg.lstsq(Matrix,nodes_Y)[0]

        # find angles and curvature along interpolation polynomial
        #b_theta_vec         = zeros(self.nPoints)
        b_curvature_vector  = zeros(self.nPoints)
        pdcx1 = polyder(coeffX,1)
        pdcy1 = polyder(coeffY,1)
        pdcx2 = polyder(coeffX,2)
        pdcy2 = polyder(coeffY,2)

        for j in range(0,self.nPoints):
            s       = s_start + j*self.ds
            dX      = polyval(pdcx1,s)
            dY      = polyval(pdcy1,s)
            ddX     = polyval(pdcx2,s)
            ddY     = polyval(pdcy2,s)
            b_curvature_vector[j] = (dX*ddY-dY*ddX)/(dX**2+dY**2)**1.5      # this calculates the curvature values for all points in the interp. interval
                                                                            # these values are going to be approximated by a polynomial!

        # calculate coefficients for curvature
        coeffCurvature  = linalg.lstsq(Matrix3rd,b_curvature_vector)[0]

        # Calculate s
        discretization = 0.001                           # discretization to calculate s

        j = s_start + arange((self.N_nodes_poly_back-1)*self.ds,(self.N_nodes_poly_back+1)*self.ds,discretization)

        x_eval      = polyval(coeffX,j)
        y_eval      = polyval(coeffY,j)
        dist_eval   = sum((self.pos*ones([size(j),2])-vstack((x_eval,y_eval)).transpose())**2,1)**0.5
        idx_s_min   = argmin(dist_eval)
        s           = j[idx_s_min]      # s = minimum distance to points between idx_min-1 and idx_min+1
        eyabs       = amin(dist_eval)   # absolute distance to curve

        # Calculate sign of y
        s0      = s - discretization
        XCurve0 = polyval(coeffX,s0)
        YCurve0 = polyval(coeffY,s0)
        dX      = polyval(polyder(coeffX,1),s)
        dY      = polyval(polyder(coeffY,1),s)

        xyVectorAngle   = arctan2(self.pos[1]-YCurve0, self.pos[0]-XCurve0)
        xyPathAngle     = arctan2(dY,dX)
        ey              = eyabs*sign(sin(xyVectorAngle-xyPathAngle))

        # Calculate epsi
        epsi = (self.psi+pi)%(2*pi)-pi-xyPathAngle
        epsi = (epsi+pi)%(2*pi)-pi

        #if s < 0.0:
        #    s = s + self.n*self.ds

        self.epsi           = epsi
        self.ey             = ey
        self.s              = s
        self.coeffCurvature = coeffCurvature
        self.s_start        = s_start


    def __init__(self):
        self.x = 0
        self.y = 0
