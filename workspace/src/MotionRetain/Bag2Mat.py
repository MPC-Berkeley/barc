import rosbag
import numpy as np
import scipy.io
import os

bag = rosbag.Bag(os.path.expanduser("~/Desktop/2017-10-06-18-21-06.bag"))


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

  if bag.get_type_and_topic_info()[1].values()[i][0] == 'sensor_msgs/Imu':
    dimIMU = bag.get_type_and_topic_info()[1].values()[i][1]
    print "Dim hedge_pos msg:", dimIMU
print types

states   = np.zeros((dimIMU, 6))
time = np.zeros((dimIMU, 1))

counter = 0
for counter, (topic, msg, t) in enumerate( bag.read_messages(topics=['/imu/data']) ) :  
  states[counter,0] = msg.angular_velocity.x
  states[counter,1] = msg.angular_velocity.y
  states[counter,2] = msg.angular_velocity.z

  states[counter,3] = msg.linear_acceleration.x
  states[counter,4] = msg.linear_acceleration.y
  states[counter,5] = msg.linear_acceleration.z

  time[counter] = float(msg.header.stamp.secs) + float(msg.header.stamp.nsecs)/1e9

scipy.io.savemat('imu.mat', mdict={'states': states, 'time':time})

bag.close()
