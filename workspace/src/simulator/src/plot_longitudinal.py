import rosbag
import numpy as np
import matplotlib.pyplot as plt
import os
import matplotlib.patches as patches
from matplotlib import animation

length = 1.738*2
height = 1.5
bag = rosbag.Bag(os.path.expanduser("~/_2017-09-17-15-35-47.bag"))


topics = bag.get_type_and_topic_info()[1].keys()
types = []
for i in range(0,len(bag.get_type_and_topic_info()[1].values())):
  types.append(bag.get_type_and_topic_info()[1].values()[i][0])

  print bag.get_type_and_topic_info()[1].values()[i][0]

  if bag.get_type_and_topic_info()[1].values()[i][0] == 'barc/ECU':
    dimEcu = bag.get_type_and_topic_info()[1].values()[i][1]
    print "Dim ECU mgs:", dimEcu

  if bag.get_type_and_topic_info()[1].values()[i][0] == 'simulator/Z_DynBkMdl':
    dimxy = bag.get_type_and_topic_info()[1].values()[i][1]
    print "Dim hedge_pos msg:", dimxy

print types

# x_list is the list of state x 
x_list     = np.array([0, 20, 20, 40,    40,    60,    60,    80,   80,   100,    100,    120,    120,   140,   140,    160, 160, 180])
theta_list = np.array([0, 0,  10, 10,    0,     0,     -10,   -10,  0,    0,      30,     30,     0,     0,     -30,    -30, 0,   0])
z_list     = np.array([0, 0,  0,  3.471, 3.471, 3.471, 3.471, 0.0,  0.0,  0.0,    0.0,    10,     10,    10,    10,     0,   0,   0])
# x_draw is the projection of state x on the x axis
x_draw = np.array([0, 20, 20, 39.696, 39.696, 59.696, 59.696, 79.392, 79.392, 99.392, 99.392, 
                   116.715, 116.715, 136.715, 136.715, 154.038, 154.038, 174.038])
x_axis = np.zeros((dimxy, 1))
z_raw = np.zeros((dimxy, 1))
theta_raw = np.zeros((dimxy, 1))
v_raw = np.zeros((dimxy, 1))

acc   = np.zeros((dimEcu, 1))
d_f   = np.zeros((dimEcu, 1))
time  = np.zeros((dimEcu, 1))

counter = 0
for counter, (topic, msg, t) in enumerate( bag.read_messages(topics=['/z_vhcl']) ) :  
  v_raw[counter] = msg.v_x
  z_raw[counter] = np.interp(msg.x, x_list, z_list)
  theta_raw[counter] = np.interp(msg.x, x_list, theta_list)
  x_axis[counter] = np.interp(msg.x, x_list, x_draw)

counter = 0
for (topic, msg, t) in bag.read_messages(topics=['/ecu']) :  
  acc[counter]  = msg.motor
  counter = counter + 1

fig = plt.figure()

ax1 = fig.add_subplot(2, 1, 1)
plt.plot(x_draw, z_list, label = 'Country road')
ax1.fill_between(x_draw, 0, z_list, facecolor='green')
plt.ylabel('Position along Z-axis [m]')
plt.xlabel('Position along X-axis [m]')
plt.title('Replay of vehicle running on the mountain')
plt.axis('equal')
ax1.set_xlim(0, 130)
ax1.set_ylim(0, 10)
plt.grid()
plt.legend(loc = 4)
patch1 = patches.Rectangle((0, 0), 0, 0, fc='r')

ax2 = fig.add_subplot(2, 1, 2)
plt.plot(x_axis, v_raw, label = 'Speed curve')
plt.xlabel('Position along X-axis [m]')
plt.ylabel('Speed of the vehicle [m/s]')
plt.axis('equal')
plt.title('Speed of vehicle vs. x-coord')
plt.grid()
plt.legend(loc = 4)
patch2 = patches.Rectangle((0, 0), 0, 0, fc='r')

def init():
    ax1.add_patch(patch1)
    ax2.add_patch(patch2)
    return patch1, patch2

def animate(i):
    patch1.set_width(length)
    patch1.set_height(height)
    # patch1.set_xy([x_axis[i] - length/2*np.cos(theta_raw[i]/180*np.pi), z_raw[i]] - length/2*np.sin(theta_raw[i]/180*np.pi))
    patch1.set_xy([x_axis[i], z_raw[i]])
    ax1.set_xlim(x_axis[i] - 10, x_axis[i] + 10)
    ax1.set_ylim(z_raw[i] - 10, z_raw[i] + 10)
    ax2.set_xlim(x_axis[i] - 10, x_axis[i] + 10)
    ax2.set_ylim(v_raw[i] - 5, v_raw[i] + 5)
    patch2.set_width(0.2)
    patch2.set_height(0.2)
    patch2.set_xy([x_axis[i] - 0.1, v_raw[i] - 0.1])
    patch1._angle = theta_raw[i]
    return patch1,patch2

anim = animation.FuncAnimation(fig, animate,
                               init_func=init,
                               frames=len(x_axis),
                               interval=20,
                               blit=False)
plt.show()
bag.close()
