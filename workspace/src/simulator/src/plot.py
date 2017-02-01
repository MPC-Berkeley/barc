import rosbag
import numpy as np
import matplotlib.pyplot as plt
import os

bag = rosbag.Bag(os.path.expanduser("~/rosbag/2017-01-31-20-50-50.bag"))


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


x_raw = np.zeros((dimxy, 1))
y_raw = np.zeros((dimxy, 1))

v_raw = np.zeros((dimxy, 1))

acc   = np.zeros((dimEcu, 1))
d_f   = np.zeros((dimEcu, 1))
time  = np.zeros((dimEcu, 1))

counter = 0
for counter, (topic, msg, t) in enumerate( bag.read_messages(topics=['/z_vhcl']) ) :  
  x_raw[counter] = msg.x
  y_raw[counter] = msg.y
  v_raw[counter] = np.sqrt( np.power(msg.v_x,2) + np.power(msg.v_x,2) )


counter = 0
for (topic, msg, t) in bag.read_messages(topics=['/ecu']) :  
  acc[counter]  = msg.motor
  d_f[counter]  = msg.servo
  counter = counter + 1


# print counter

plt.figure(1)
plt.plot(x_raw, y_raw,'-o')
plt.ylabel('Position along Y-axis [m]')
plt.xlabel('Position along X-axis [m]')

plt.figure(2)
plt.plot(v_raw)
plt.ylabel('Velocity [m/s]')


plt.figure(3)
plt.plot(time, d_f, '-o')
plt.ylabel('Steering [rad]')


plt.figure(4)
plt.plot(time, acc, '-o')
plt.ylabel('Acceleration [m/s^2]')

plt.show()

bag.close()
