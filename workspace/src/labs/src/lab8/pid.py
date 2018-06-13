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
    def __init__(self, P=1.0, I=1.0, D=0, dt = 0.1, Integrator_max=100, Integrator_min=-100):
        self.Kp             = P             # proportional gain
        self.Ki             = I             # integral gain
        self.Kd             = D             # derivative gain
        self.dt             = dt

        self.set_point      = 0             # reference point
        self.e              = 0

        self.e_int          = 0
        self.int_e_max      = Integrator_max
        self.int_e_min      = Integrator_min

        self.current_value  = 0


    def update(self,current_value):
        self.current_value = current_value

        e_t         = self.set_point - current_value
        de_t        = ( e_t - self.e)/self.dt

        self.e_int  = self.e_int + e_t * self.dt
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

    def setPID(self,P=1,I=1,D=0):
        self.Kp = P
        self.Ki = I
        self.Kd = D
        return self.e

    def setTimeStep(self, dt):
        self.dt = dt

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
    pid.setTimeStep(dt)

    for i in range(n-1):
        u       = pid.update(x[i])
        x[i+1] = fx(x[i],u, dt)

    plt.plot(x)
    plt.show()
