from numpy import *
import matplotlib.pyplot as plt

x = array([0])           # starting point
y = array([0])
ds = 0.06
theta = 0
d_theta = 0

N = 40

halfcircle = sum(arange(1,N+1,1))

for i in range(0,220):
    if i < 10:
        d_theta = 0
    elif i < 51:
        d_theta = d_theta + pi/(2*halfcircle+N)
    elif i < 90:
        d_theta = d_theta - pi/(2*halfcircle+N)
    elif i < 120:
        d_theta = 0#d_theta + pi / halfcircle
    elif i < 161:
        d_theta = d_theta + pi/(2*halfcircle+N)
    elif i < 200:
        d_theta = d_theta - pi/(2*halfcircle+N)
    else:
        d_theta = 0
    theta = theta + d_theta
    print(d_theta)
    #print(theta)
    x = hstack((x, x[-1] + cos(theta)*ds))
    y = hstack((y, y[-1] + sin(theta)*ds))

plt.plot(x,y,'-o')
plt.axis('equal')
plt.show()