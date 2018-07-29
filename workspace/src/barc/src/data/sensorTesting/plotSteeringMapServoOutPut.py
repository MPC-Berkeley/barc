
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
		print "if msg.servo >= ", y[i][1],":"
	elif i != len(PWM_interval_float)-2:
		print "elif msg.servo <= ", y[i][0], " and msg.servo >= ", y[i][1],":"
	else:
		print "elif msg.servo <= ", y[i][0], ":"
	
	print "\tself.servo_pwm = (float(msg.servo) + ", -c[i], ") / ", m[i]

plt.show()    


# Now fit servo output
ax = plt.figure(3)
plt.plot(fbk_servo, steering_his, 'o')
plt.legend()
plt.ylabel('steering')
plt.xlabel('Servo Output Steering')
plt.xlim((min(fbk_servo), max(fbk_servo)))

mi, ci = fit(min(fbk_servo), max(fbk_servo), fbk_servo, steering_his)
xi = [i for i in np.arange(min(fbk_servo), max(fbk_servo),1.0)]
yi = [mi*float(a)+ci for a in xi]
plt.plot(xi, yi, '-rs')

print "Equation is: tangle_rad = ", mi ,"*fbk_srv + ", ci
plt.show()

ax = plt.figure(4)
plt.plot(fbk_servo, steering_his, 'o')
plt.legend()
plt.ylabel('steering')
plt.xlabel('Servo Output Steering')
plt.xlim((min(fbk_servo), max(fbk_servo)))
low_lim_1 = 220
up__lim_1 = 320
mi_1, ci_1 = fit(low_lim_1, up__lim_1, fbk_servo, steering_his)
xi = [i for i in np.arange(low_lim_1, up__lim_1,1.0)]
yi = [mi_1*float(a)+ci_1 for a in xi]
plt.plot(xi, yi, '-bs')

low_lim_2 = up__lim_1
up__lim_2 = 350
mi_2, ci_2 = fit(low_lim_2, up__lim_2, fbk_servo, steering_his)
xi = [i for i in np.arange(low_lim_2, up__lim_2,1.0)]
yi = [mi_2*float(a)+ci_2 for a in xi]
plt.plot(xi, yi, '-rs')

low_lim_3 = up__lim_2 
up__lim_3 = 420
mi_3, ci_3 = fit(low_lim_3, up__lim_3, fbk_servo, steering_his)
xi = [i for i in np.arange(low_lim_3, up__lim_3,1.0)]
yi = [mi_3*float(a)+ci_3 for a in xi]
plt.plot(xi, yi, '-ks')
plt.show()

print "if fbk_srv <= ", up__lim_1, ":"
print "\tangle_rad = ", mi_1 ,"*fbk_srv + ", ci_1
print "elif fbk_srv <= ", up__lim_2, " and fbk_srv >= ", low_lim_2,":"
print "\tangle_rad = ", mi_2 ,"*fbk_srv + ", ci_2
print "if fbk_srv >= ", low_lim_3, ":"
print "\tangle_rad = ", mi_3 ,"*fbk_srv + ", ci_3
