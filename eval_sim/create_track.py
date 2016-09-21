from numpy import *
import matplotlib.pyplot as plt

x = array([0])           # starting point
y = array([0])
ds = 0.06
theta = 0
d_theta = 0

N = 40

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


theta = array([0])

# theta = add_curve(theta,10,0)
# theta = add_curve(theta,50,-2*pi/3)
# #theta = add_curve(theta,30,0)
# theta = add_curve(theta,70,pi)
# theta = add_curve(theta,60,-5*pi/6)
# theta = add_curve(theta,10,0)
# theta = add_curve(theta,30,-pi/2)
# theta = add_curve(theta,40,0)
# theta = add_curve(theta,20,-pi/4)
# theta = add_curve(theta,20,pi/4)
# theta = add_curve(theta,50,-pi/2)
# theta = add_curve(theta,22,0)
# theta = add_curve(theta,30,-pi/2)
# theta = add_curve(theta,14,0)

theta = add_curve(theta,50,0)
theta = add_curve(theta,100,-pi)
theta = add_curve(theta,100,0)
theta = add_curve(theta,100,-pi)
theta = add_curve(theta,49,0)

for i in range(0,size(theta)):
    x = hstack((x, x[-1] + cos(theta[i])*ds))
    y = hstack((y, y[-1] + sin(theta[i])*ds))

print x
print y
plt.plot(x,y,'-o')
plt.axis('equal')
plt.grid()
plt.show()