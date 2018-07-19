
import sys
import pickle
import matplotlib.pyplot as plt    
import numpy as np
import pdb

file_data = open(sys.path[0]+'/steeringMap_BARC.obj', 'rb')
PWMsteering_his = pickle.load(file_data)
steering_his = pickle.load(file_data)
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


ax = plt.figure(1)
plt.plot(PWMsteering_his, steering_his, 'o')
plt.legend()
plt.ylabel('steering')
plt.xlabel('PWM')
plt.xlim((50, 150))

PWM_interval = [0, 83, 120]
PWM_interval_float = sorted([float(x) for x in PWM_interval])
print PWM_interval_float
m = []
c = []
x = []
y = []
for i in range(0, len(PWM_interval_float)-1):
	mi, ci = fit(PWM_interval_float[i], PWM_interval_float[i+1], PWMsteering_his, steering_his)
	xi = [PWM_interval_float[i], PWM_interval_float[i+1]]
	yi = [mi*float(a)+ci for a in xi]
	plt.plot(xi, yi, '-bs')

	m.append(mi)
	c.append(ci)
	x.append(xi)
	y.append(yi)

for i in range(0, len(PWM_interval_float)-1):
	if i == 0:
		print "if msg.servo <= ", y[i][1],":"
	elif i != len(PWM_interval_float)-2:
		print "elif msg.servo >= ", y[i][0], " and msg.servo <= ", y[i][1],":"
	else:
		print "elif msg.servo >= ", y[i][0], ":"
	
	print "\tself.servo_pwm = (float(msg.servo) + ", -c[i], ") / ", m[i]


plt.show()

ax = plt.figure(2)
plt.plot(PWMsteering_his, steering_his, 'o')
plt.legend()
plt.ylabel('steering')
plt.xlabel('PWM')
plt.xlim((50, 150))

PWM_interval = list(set(PWMsteering_his))
PWM_interval_float = sorted([float(x) for x in PWM_interval])
print PWM_interval_float
m = []
c = []
x = []
y = []
for i in range(0, len(PWM_interval_float)-1):
	mi, ci = fit(PWM_interval_float[i], PWM_interval_float[i+1], PWMsteering_his, steering_his)
	xi = [PWM_interval_float[i], PWM_interval_float[i+1]]
	yi = [mi*float(a)+ci for a in xi]
	plt.plot(xi, yi, '-bs')

	m.append(mi)
	c.append(ci)
	x.append(xi)
	y.append(yi)

for i in range(0, len(PWM_interval_float)-1):
	if i == 0:
		print "if msg.servo <= ", y[i][1],":"
	elif i != len(PWM_interval_float)-2:
		print "elif msg.servo >= ", y[i][0], " and msg.servo <= ", y[i][1],":"
	else:
		print "elif msg.servo >= ", y[i][0], ":"
	
	print "\tself.servo_pwm = (float(msg.servo) + ", -c[i], ") / ", m[i]

plt.show()    


# Now fit servo output
ax = plt.figure(3)
plt.plot(PWMsteering_his, steering_his, 'o')
plt.legend()
plt.ylabel('steering')
plt.xlabel('PWM')
plt.xlim((min(fbk_servo), max(fbk_servo)))

mi, ci = fit(min(fbk_servo), max(fbk_servo), fbk_servo, steering_his)
xi = [i for i in np.arange(min(fbk_servo), max(fbk_servo),1.0)]
yi = [mi*float(a)+ci for a in xi]
plt.plot(xi, yi, '-bs')

print "Equation is: ", mi ,"*fbk_servo + ", ci
plt.show()