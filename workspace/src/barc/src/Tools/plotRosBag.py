import rosbag
import numpy as np
import matplotlib.pyplot as plt
import pdb

Experiment_1 = True

if Experiment_1 == True:
	bag = rosbag.Bag('/home/ugo/rosbag/Experiment_1/2018-07-12-11-04-13.bag')
	bagMoCap = rosbag.Bag('/home/ugo/rosbag/Experiment_1/2018-07-12-11-04-08.bag')
else:
	bag = rosbag.Bag('/home/ugo/rosbag/Experiment_2/2018-07-12-11-19-26.bag')
	bagMoCap = rosbag.Bag('/home/ugo/rosbag/Experiment_2/ThirdRunForMark.bag')


psi_bag_MoCap = []
time_bag_MoCap = []

for topic, msg, time in bagMoCap.read_messages():
	if topic == '/mocap_output1':
		# pdb.set_trace()
		# if offSet == 0.0:
		#   time_bag.append(0)
		#   offSet = msg.header.stamp.to_sec()
		# else:
		# print msg
		# pdb.set_trace()
		time_bag_MoCap.append(msg.header.stamp.to_sec())
		psi_bag_MoCap.append(msg.attyaw)

psi_bag = []
time_bag = []
YawOffSet = 0.0
ChangeCoordinates = 0.102
for topic, msg, time in bag.read_messages():
	if topic == '/pos_info':
		# pdb.set_trace()
		if YawOffSet == 0.0:
			YawOffSet = msg.psi
			angleRap = msg.psi+(psi_bag_MoCap[0]-YawOffSet)
		else:
			angle = msg.psi+(psi_bag_MoCap[0]-YawOffSet)
			angleRap = np.unwrap([0,angle])[1] 
			if angleRap > 6.28:
				pdb.set_trace()

		# pdb.set_trace()
		time_bag.append(msg.header.stamp.to_sec())
		psi_bag.append(angleRap)
			
pdb.set_trace()

plt.figure(1)
plt.plot(time_bag, [i+ChangeCoordinates for i in psi_bag],'-o', label="Estimated Yaw")
plt.plot(time_bag_MoCap, psi_bag_MoCap,'-*', label="MoCap Yaw")
plt.legend()

plt.ylabel('Yaw')
plt.xlabel('Time')

plt.show()

bag.close()
