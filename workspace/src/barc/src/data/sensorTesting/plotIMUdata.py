
import sys
import pickle
import matplotlib.pyplot as plt    

file_data = open(sys.path[0]+'/IMU_oldBARC.obj', 'rb')
time = pickle.load(file_data)
yawRate = pickle.load(file_data)
yaw_raw = pickle.load(file_data)  
yaw_int = pickle.load(file_data) 
roll_raw = pickle.load(file_data)  
pitch_raw = pickle.load(file_data) 
ax = pickle.load(file_data) 
ay = pickle.load(file_data)  
az = pickle.load(file_data) 

file_data.close()
plt.figure(1)
plt.subplot(4,1,1)
plt.plot(time, yawRate, '-')
plt.subplot(4,1,2)
plt.plot(time, yaw_raw, '-r', label="raw")
plt.plot(time, yaw_int, '-g', label="integrated")
plt.legend()
plt.subplot(4,1,3)
plt.plot(time, roll_raw, '-r', label="roll_raw")
plt.plot(time, pitch_raw, '-g', label="pitch_raw")
plt.legend()
plt.subplot(4,1,4)
plt.plot(time, ax, '-r', label="ax")
plt.plot(time, ay, '-g', label="ay")
plt.plot(time, az, '-k', label="az")
plt.legend()

plt.show()

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
    

