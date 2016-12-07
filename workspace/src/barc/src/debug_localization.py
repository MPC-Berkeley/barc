from Localization_helpers import Localization
from numpy import eye, array, zeros, diag, unwrap, tan, cos, sin, vstack, linalg, append
from numpy import ones, polyval, delete, size, empty, linspace
import math
import matplotlib.pyplot as plt

l = Localization()
l.create_track()

plt.plot(l.nodes[0,:],l.nodes[1,:],"r-o")
plt.grid('on')
plt.axis('equal')
plt.show()


n = size(l.nodes[0,:])
#c = empty([n,10])
ss = empty(n)
for i in range(0,n):
    l.set_pos(l.nodes[0,i], l.nodes[1,i], 0.0,0.0,0.0,0.0)
    l.find_s()
    s = linspace(l.s,l.s+2.0,5)
    ss[i] = l.s
    #c[i,:] = polyval(l.coeffCurvature,s)
    c = polyval(l.coeffCurvature,s)
    plt.plot(s,c)
    print l.s
    #print c

#plt.plot(c)
plt.show()