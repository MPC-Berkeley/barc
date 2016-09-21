from numpy import *
import matplotlib.pyplot as plt

x = array([0])           # starting point
y = array([0])
ds = 0.06
theta = 0
d_theta = 0

N = 40

def create_bezier(p0,p1,p2,dt):
    t = arange(0,1+dt,dt)[:,None]
    p = (1-t)**2*p0 + 2*(1-t)*t*p1+t**2*p2
    return p

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

plt.plot(p[:,0],p[:,1],'-',p0[:,0],p0[:,1],'*')
plt.grid()
plt.axis('equal')
plt.show()
