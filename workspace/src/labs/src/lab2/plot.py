import rosbag
import numpy as np
import matplotlib.pyplot as plt
import os
import matplotlib.patches as patches

bag = rosbag.Bag(os.path.expanduser("~/FILENAMEHERE.bag"))


topics = bag.get_type_and_topic_info()[1].keys()
types = []
for i in range(0,len(bag.get_type_and_topic_info()[1].values())):
  types.append(bag.get_type_and_topic_info()[1].values()[i][0])
  if bag.get_type_and_topic_info()[1].values()[i][0] == 'barc/ECU':
    dimEcu = bag.get_type_and_topic_info()[1].values()[i][1]
  if bag.get_type_and_topic_info()[1].values()[i][0] == 'labs/Z_DynBkMdl':
    dimxy = bag.get_type_and_topic_info()[1].values()[i][1]


x_raw = np.zeros((dimxy, 1))
v_raw = np.zeros((dimxy, 1))
v_des = 8*np.ones((dimxy,1))

counter = 0
for counter, (topic, msg, t) in enumerate( bag.read_messages(topics=['/z_vhcl']) ) :  
  x_raw[counter] = msg.x
  v_raw[counter] = msg.v_x

plt.figure(1)
plt.plot(x_raw, v_raw, label = 'Actual Velocity')
plt.plot(x_raw, v_des, label = 'Desired Velocity')
plt.ylabel('Velocity [m/s]')
plt.ylim((0,12))
plt.xlabel('Longitudinal position [m]')
plt.title('Longitudinal Velocity Tracking')
plt.legend()
plt.show()

bag.close()
