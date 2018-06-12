
import sys
import pickle
import matplotlib.pyplot as plt    

file_data = open(sys.path[0]+'/IMU_oldBARC.obj', 'rb')
time = pickle.load(file_data)
yawRate = pickle.load(file_data)
yaw_raw = pickle.load(file_data)  
yaw_int = pickle.load(file_data)    
file_data.close()
plt.figure(1)
plt.subplot(2,1,1)
plt.plot(time, yawRate, '-')
plt.subplot(2,1,2)
plt.plot(time, yaw_raw, '-r')
plt.plot(time, yaw_int, '-g')

file_data = open(sys.path[0]+'/IMU_newBARC.obj', 'rb')
time = pickle.load(file_data)
yawRate = pickle.load(file_data)
yaw_raw = pickle.load(file_data)  
yaw_int = pickle.load(file_data)    
file_data.close()
plt.figure(2)
plt.subplot(2,1,1)
plt.plot(time, yawRate, '-')
plt.subplot(2,1,2)
plt.plot(time, yaw_raw, '-r')
plt.plot(time, yaw_int, '-g')

plt.show()
    

