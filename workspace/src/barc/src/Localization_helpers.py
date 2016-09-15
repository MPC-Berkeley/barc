from numpy import *#sin, cos, tan, arctan, array, dot, linspace, pi, ones, zeros, sum, amin, argmin
import scipy.optimize

def create_circle(rad,n,c):
	ang = linspace(0,2*pi,n)
	x = rad*cos(ang)+c[0]
	y = rad*sin(ang)+c[1]
	return array([x,y])

class Localization:
	n 					= 0						# number of nodes
	c 					= 0						# center of circle (in case of circular trajectory)
	pos 				= 0						# current position
	psi 				= 0						# current orientation
	nodes 				= 0						# all nodes are saved in a matrix
	N_nodes_poly_back 	= 10					# number of nodes behind current position
	N_nodes_poly_front 	= 30					# number of nodes in front
	ds 					= 0						# distance between nodes
	nPoints 			= N_nodes_poly_front+N_nodes_poly_back+1	# number of points for interpolation in total
	OrderXY 			= 6						# order of x-y-polynomial interpolation
	OrderThetaCurv 		= 4						# order of theta interpolation
	closed				= True					# open or closed trajectory?

	coeffX = 0
	coeffY = 0
	coeffTheta = 0
	coeffCurvature = 0

	s = 0					# distance from s_start to current closest node (idx_min)
	s_start = 0				# distance along path from first node to start node (which is N_nodes_poly_back behind current closest node)
	ey = 0					# lateral distance to path
	epsi = 0				# error in psi (between current psi and trajectory tangent)
	v = 0					# current velocity (not necessary for these calculations but for MPC)

	def create_circle(self,rad=1.0,n=100,c=array([0,0])):			# note: need to make sure that all points are equidistant!
		ang = linspace(0,2*pi-2*pi/n,n)
		#print(ang)
		x = rad*cos(ang)+c[0]
		y = rad*sin(ang)+c[1]
		self.nodes 	= array([x,y])
		self.n 		= n
		self.c 		= c
		self.rad 	= rad
		#self.ds 	= rad*2*pi/n
		self.ds 	= 2*rad*tan(2*pi/n/2)
	def create_racetrack(self,L=1.0,b=1.0,ds=0.5,c=array([0,0]),ang=0):		# problem: points are not equidistant at connecing points
		x = linspace(0,L/2.0,5)#arange(0,L/2.0,ds)											# otherwise: would create a racetrack with parallel lines
		s = size(x)
		da = 2*pi/360*5
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

		R = array([[cos(ang),-sin(ang)],[sin(ang),cos(ang)]])		# Rotation Matrix

		nodes = dot(R,array([x,y]))+(c*ones([2,s_tot]).T).T

		self.nodes = nodes
		self.n = size(x)
		self.ds = ds

	def create_ellipse(self,L_a,L_b,n,c=array([0,0])):
		ang = linspace(0,2*pi,n)
		x = L_a*cos(ang)+c[0]
		y = L_b*sin(ang)+c[1]
		self.nodes 	= array([x,y])
		self.n 		= n
		self.c 		= c

	def prepare_trajectory(self,ds_in):		# brings trajectory to correct format (equidistant nodes with distances around ds)
		x = self.nodes[0]
		y = self.nodes[1]
		n = size(x)								# number of nodes
		ds = (diff(x)**2+diff(y)**2)**0.5		# distances between nodes
		#if self.closed:
		#	ds = hstack((ds,((x[n-1]-x[0])**2+(y[n-1]-y[0])**2)**0.5))
		s = hstack((0,cumsum(ds)))
		length = sum(ds)
		print "Track length = %fm"%length
		dsn = ds_in						# optimal new step size (might be calculated externally for now)
		rem = lambda x:mod(length,x)    # function to calculate optimal step size (for evenly distributed steps)
		sol = scipy.optimize.fmin(func=rem,x0=dsn)
		dsn = sol[0]
		sn = arange(0,length,dsn)		# new steps
		xn = interp(sn,s,x)
		yn = interp(sn,s,y)

		self.nodes = array([xn,yn])
		self.ds = dsn
		self.n = size(xn)


	def set_pos(self,x,y,psi,v):
		self.pos = array([x,y])
		self.psi = psi
		self.v = v

	def find_s(self):
		dist = sum((self.pos*ones([self.n,2])-self.nodes.transpose())**2,1)**0.5	# distance of pos to all nodes
		#mindist = amin(dist)		# minimum distance
		idx_min = argmin(dist)	# index of minimum distance
		n = self.n
		nPoints = self.nPoints
		# Use closest node to determine start and end of polynomial approximation
		idx_start = idx_min - self.N_nodes_poly_back
		idx_end   = idx_min + self.N_nodes_poly_front
		if self.closed == True:
			if idx_start<0:
				nodes_X = hstack((self.nodes[0,n+idx_start:n],self.nodes[0,0:idx_end+1]))
				nodes_Y = hstack((self.nodes[1,n+idx_start:n],self.nodes[1,0:idx_end+1]))
				idx_start = n+idx_start
			elif idx_end>n-1:
				nodes_X = hstack((self.nodes[0,idx_start:n],self.nodes[0,0:idx_end+1-n]))
				nodes_Y = hstack((self.nodes[1,idx_start:n],self.nodes[1,0:idx_end+1-n]))
			else:
				nodes_X = self.nodes[0,idx_start:idx_end+1]
				nodes_Y = self.nodes[1,idx_start:idx_end+1]
		else:
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
		Matrix = zeros([self.nPoints,self.OrderXY+1])
		for i in range(0,self.nPoints):
			for k in range(0,self.OrderXY+1):
				Matrix[i,self.OrderXY-k] = (i*self.ds)**k

		# curvature matrix
		Matrix3rd = zeros([self.nPoints,self.OrderThetaCurv+1])
		for i in range(0,self.nPoints):
			for k in range(0,self.OrderThetaCurv+1):
				Matrix3rd[i,self.OrderThetaCurv-k] = (i*self.ds)**k
				
		# Solve system of equations to find polynomial coefficients (for x-y)
		self.coeffX = linalg.lstsq(Matrix,nodes_X)[0]
		self.coeffY = linalg.lstsq(Matrix,nodes_Y)[0]

		# find angles and curvature along interpolation polynomial
		b_theta_vec 		= zeros(self.nPoints)
		b_curvature_vector 	= zeros(self.nPoints)

		for j in range(0,self.nPoints):
			s 		= j*self.ds
			dX 		= polyval(polyder(self.coeffX,1),s)
			dY 		= polyval(polyder(self.coeffY,1),s)
			ddX 	= polyval(polyder(self.coeffX,2),s)
			ddY 	= polyval(polyder(self.coeffY,2),s)
			angle 	= arctan2(dY,dX)
			if j>1:
				if angle - b_theta_vec[j-1] > pi:
					angle = angle - 2*pi
				elif angle - b_theta_vec[j-1] < -pi:
					angle = angle + 2*pi

			b_theta_vec[j] = angle
			b_curvature_vector[j] = (dX*ddY-dY*ddX)/(dX**2+dY**2)**1.5

		# calculate coefficients for Theta and curvature
		coeffTheta 		= linalg.lstsq(Matrix3rd,b_theta_vec)[0]
		coeffCurvature 	= linalg.lstsq(Matrix3rd,b_curvature_vector)[0]

		# Calculate s
		discretization = 0.01
		s_idx_start = max(0,idx_min-1)
		if self.closed and s_idx_start-idx_start<0:
			s_idx_start = s_idx_start + n
		j 			= arange((s_idx_start-idx_start)*self.ds,(s_idx_start-idx_start+2)*self.ds,discretization)
		x_eval 		= polyval(self.coeffX,j)
		y_eval 		= polyval(self.coeffY,j)
		dist_eval 	= sum((self.pos*ones([size(j),2])-vstack((x_eval,y_eval)).transpose())**2,1)**0.5
		idx_s_min 	= argmin(dist_eval)
		s 			= j[idx_s_min]		# s = minimum distance to points between idx_min-1 and idx_min+1
		eyabs 		= amin(dist_eval)	# absolute distance to curve

		# Calculate sign of y
		s0 		= s - discretization
		XCurve0 = polyval(self.coeffX,s0)
		YCurve0 = polyval(self.coeffY,s0)
		XCurve 	= polyval(self.coeffX,s)
		YCurve 	= polyval(self.coeffY,s)
		dX 		= polyval(polyder(self.coeffX,1),s)
		dY 		= polyval(polyder(self.coeffY,1),s)

		xyVectorAngle 	= arctan2(self.pos[1]-YCurve0, self.pos[0]-XCurve0)
		xyPathAngle 	= arctan2(dY,dX)
		ey 				= eyabs*sign(sin(xyVectorAngle-xyPathAngle))

		# Calculate epsi
		epsi = (self.psi+pi)%(2*pi)-pi-xyPathAngle
		epsi = (epsi+pi)%(2*pi)-pi
		#if abs(epsi) > pi/2:
		#	if epsi < pi/2:
		#		epsi = epsi + 2*pi
		#	else:
		#		epsi = epsi - 2*pi
        #epsi = (self.psi+pi)%(2*pi)+pi-xyPathAngle
		self.epsi 			= epsi
		self.ey 			= ey
		self.s 				= s
		self.coeffTheta 	= coeffTheta
		self.coeffCurvature = coeffCurvature
		self.s_start = idx_start*self.ds


	def __init__(self):
		self.x = 0
		self.y = 0
