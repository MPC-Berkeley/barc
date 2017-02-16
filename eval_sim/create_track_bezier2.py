from numpy import *
import matplotlib.pyplot as plt

x = array([0])           # starting point
y = array([0])
ds = 0.06
theta = 0
d_theta = 0

N = 40

def create_bezier(p0,p1,p2,p3,dt):
    t = arange(0,1+dt,dt)[:,None]
    p = (1-t)**3*p0 + 3*(1-t**2)*t*p1+3*(1-t)*t**2*p2+t**3*p3
    return p

p0 = array([[0,0],
            [0,-1],
            [0,-2]])
p1 = array([[1,0],
            [-1,-1]])

p2 = 2*p0[1,:] - p1[1,:]
p3 = p0[1,:]
for i in range(1,size(p0,0)-2):
    p3 = vstack((p3,p0[i,:]))
    p2 = vstack((p2,2*p0[i+1]-p1[i+1]))

p = p0[0,:]
for i in range(0,size(p0,0)-1):
    p = vstack((p,create_bezier(p0[i,:],p1[i,:],p2[i,:],p3[i,:].0.01)))

plt.plot(p[:,0],p[:,1],'-',p0[:,0],p0[:,1],'*')
plt.grid()
plt.axis('equal')
plt.show()
