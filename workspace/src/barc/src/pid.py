#!/usr/bin/env python

# ---------------------------------------------------------------------------
# Licensing Information: You are free to use or extend these projects for
# education or reserach purposes provided that (1) you retain this notice
# and (2) you provide clear attribution to UC Berkeley, including a link
# to http://barc-project.com
#
# Attibution Information: The barc project ROS code-base was developed
# at UC Berkeley in the Model Predictive Control (MPC) lab by Jon Gonzales
# (jon.gonzales@berkeley.edu). The cloud services integation with ROS was developed
# by Kiet Lam  (kiet.lam@berkeley.edu). The web-server app Dator was
# based on an open source project by Bruce Wootton
# ---------------------------------------------------------------------------

class PID:
    def __init__(self, P=2.0, I=0.0, D=0, de=0, e_int = 0, Integrator_max=500, Integrator_min=-500):
        self.Kp             = P             # proportional gain
        self.Ki             = I             # integral gain
        self.Kd             = D             # derivative gain

        self.set_point      = 0             # reference point
        self.e              = 0

        self.e_int          = 0
        self.int_e_max      = Integrator_max
        self.int_e_min      = Integrator_min

        self.current_value  = 0

    def update(self,current_value, dt):
        self.current_value = current_value

        e_t         = self.set_point - current_value
        de_t        = ( e_t - self.e)/dt

        self.e_int  = self.e_int + e_t * dt
        if self.e_int > self.int_e_max:
            self.e_int = self.int_e_max
        elif self.e_int < self.int_e_min:
            self.e_int = self.int_e_min

        P_val  = self.Kp * e_t
        I_val  = self.Ki * self.e_int
        D_val  = self.Kd * de_t

        PID = P_val + I_val + D_val
        self.e      = e_t

        return PID

    def setPoint(self,set_point):
        self.set_point    = set_point
        # reset error integrator
        self.e_int        = 0
        # reset error, otherwise de/dt will skyrocket
        self.e            = set_point - self.current_value

    def setKp(self,P):
        self.Kp=P

    def setKi(self,I):
        self.Ki=I

    def setKd(self,D):
        self.Kd=D

    def getPoint(self):
        return self.set_point

    def getError(self):
        return self.e

#%% Example function
def fx(x, u, dt):
    x_next = x +  (3*x + u) * dt
    return x_next

#%% Test script to ensure program is functioning properly
if __name__ == "__main__":

    from numpy import zeros
    import matplotlib.pyplot as plt

    n       = 200
    x       = zeros(n)
    x[0]    = 20
    pid     = PID(P=3.7, I=5, D=0.5)
    dt = 0.1

    for i in range(n-1):
        u       = pid.update(x[i], dt)
        x[i+1] = fx(x[i],u, dt)

    plt.plot(x)
    plt.show()
