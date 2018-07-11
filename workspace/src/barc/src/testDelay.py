import numpy as np 
import matplotlib.pyplot as plt    
import pdb

print "Here"
df1 = np.zeros(100)
df2 = np.ones(100)
print df1, df2
df = np.hstack((df1, df2))

delayed = np.zeros(df.shape[0])

servo_inp = 0.0
T  = 0.01
Tf = 0.05
for i in range(1, df.shape[0]):
	delayed[i] = (1 - T / Tf) * delayed[i-1] + ( T / Tf )*df[i-1]

time = np.arange(0, df.shape[0]) * T
plt.figure()
plt.plot(time, df, '-ob')
plt.plot(time, delayed, '-*r')
plt.show()