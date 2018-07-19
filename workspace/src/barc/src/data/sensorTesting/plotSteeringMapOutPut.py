
import sys
import pickle
import matplotlib.pyplot as plt    
import numpy as np
import pdb

file_data = open(sys.path[0]+'/steeringMap_BARC.obj', 'rb')
PWMsteering_his = pickle.load(file_data)
steering_his = pickle.load(file_data)
PWMsteering_his_servo = pickle.load(file_data)
fbk_servo = pickle.load(file_data)

def fit(minPWM, maxPWM, PWMsteering_his, steering_his):
	minPWM = minPWM - 0.1
	maxPWM = maxPWM + 0.1

	Elements = len([1 for i in PWMsteering_his if ((float(i) >= minPWM) and (float(i) <= maxPWM))])
	A = np.ones((Elements,2))
	y = np.zeros(Elements)

	counter = 0
	for i in range(0, len(PWMsteering_his)):
		if ((float(PWMsteering_his[i]) >= minPWM) and (float(PWMsteering_his[i]) <= maxPWM)):
			A[counter,0] = PWMsteering_his[i]
			y[counter]   = steering_his[i]
			counter += 1

	m, c = np.linalg.lstsq(A, y)[0]
	
	return m, c


PWM_interval = list(set(PWMsteering_his_servo))
PWM_interval_float = sorted([float(x) for x in PWM_interval])

# print PWM_interval
# fdk_servo_mean = []
# PWM_servo_mean = []
# for PWM in PWM_interval_float:
# 	tot = 0.0
# 	counterTot = 0.0
# 	for i in range(0, len(PWMsteering_his_servo)):
# 		print PWM, PWMsteering_his_servo[i]
# 		if float(PWMsteering_his_servo[i]) == PWM:
# 			tot = tot + fbk_servo[i]
# 			counterTot = counterTot + 1.0

# 	meanValue = tot/counterTot
# 	fdk_servo_mean.append(meanValue)
# 	PWM_servo_mean.append(PWM)

measuredAngle_deg = [-20.0, -18.0, -14.0, -11.0, -7.5, -5.5, -2.0, -1.0,   0.0,     3.5,   5.0,   6.5,  9.0, 11.0, 13.0, 16.0]
measuredAngle = []
for x in measuredAngle_deg:
	measuredAngle.append(x*3.14/180.0)

PWM_measuredAngles = [60.0, 65.0, 70.0, 75.0, 80.0, 85.0, 90.0, 95.0, 97.0, 100.0, 105.0, 110.0,115.0,120.0,125.0,130.0]

listMeasuredAngles = []

for i in range(0, len(PWMsteering_his_servo)):
	for j in range(0, len(PWM_measuredAngles)):
		if float(PWMsteering_his_servo[i]) == PWM_measuredAngles[j]:
			listMeasuredAngles.append(measuredAngle[j])

pdb.set_trace()
ax = plt.figure(2)
plt.plot(listMeasuredAngles, fbk_servo, 'o')
plt.legend()
plt.ylabel('fbk_servo')
plt.xlabel('listMeasuredAngles')
# plt.xlim((50, 150))

m = []
c = []
x = []
y = []
for i in range(0, len(measuredAngle)-1):
	mi, ci = fit(measuredAngle[i], measuredAngle[i+1], listMeasuredAngles, fbk_servo)
	xi = [measuredAngle[i], measuredAngle[i+1]]
	yi = [mi*float(a)+ci for a in xi]
	plt.plot(xi, yi, '-bs')

	m.append(mi)
	c.append(ci)
	x.append(xi)
	y.append(yi)

for i in range(0, len(measuredAngle)-1):
	if i == 0:
		print "if fbk_servo <= ", y[i][1],":"
	elif i != len(measuredAngle)-2:
		print "elif fbk_servo >= ", y[i][0], " and fbk_servo <= ", y[i][1],":"
	else:
		print "elif fbk_servo >= ", y[i][0], ":"
	
	print "\tvalue = (float(fbk_servo) + ", -c[i], ") / ", m[i]

plt.show()    

