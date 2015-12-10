# -*- coding: utf-8 -*-
"""
IMU sensor model 
inputs: noisy signal (a_x, a_y, psi)
returns: filtered/clean signal (v_x, v_y, psi)

Created on Mon Sep 28 2015
@author: jgon13
"""
from numpy import array, dot, cos, sin, zeros, roll, pi

class imuSignal:
    def __init__(self, y0 = 0, a = 1.0, n = 200, method = None):
        # initialize system
        self.filter         = method        # filtering method
        self.alpha          = a             # smoothing parameter
        self.n              = float(n)      # size of (weighted) moving average filter
        
        self.y_t            = y0            # current (filtered) signal
        self.y_t_RAW        = y0            # current un-filtered signal
        self.y_inertia      = 0             # inertia of signal history
        self.y_hist         = zeros(n)      # signal history
        self.mvsig          = False         # multivariate signal
        
        # vector for signal history
        if isinstance(y0, list):
            self.mvsig          = True
            self.y_t            = array(y0)
            self.y_hist         = zeros((len(y0), n))
            self.y_inertia      = zeros(len(y0))
        
        self.wgt            = array(range(n))       # moving average weights
        self.wgt_sum        = float(sum(self.wgt))
        
        # ensure a valid filtering method
        if isinstance(method,str):
            method          = method.lower()
            self.filter     = method
        if method not in [None, 'lp', 'mvg', 'wmvg']:
            raise ValueError('Please enter valid filtering method')
    
    def update(self, y_new):
        self.y_t_RAW = y_new
        
        if self.filter == None:
            self.y_t    = y_new
            
        if self.filter == 'lp':
            self.lowpass(y_new)
            
        if self.filter == 'mvg':
            self.moving_avg(y_new)
            
        if self.filter == 'wmvg':
            self.wgt_moving_avg(y_new)
    
    def lowpass(self, y_new):
        """
        Low-pass filter
        y_new       := new measurement contribution
        y_inertia   := inertia from pervious sensor outputs
        alpha       := filtering factor, (heavy filter)  0 <= a <= 1 (no filter)
        """
        if not self.mvsig:
            y_new           = y_new
            self.y_t        = self.alpha*y_new + (1-self.alpha)*self.y_inertia
            self.y_inertia  = self.y_t
        else:
            y_new           = array(y_new)
            self.y_t        = self.alpha*y_new + (1-array(self.alpha))*self.y_inertia
            self.y_inertia  = self.y_t
            
    def moving_avg(self, y_new):
        
        if not self.mvsig:
            self.y_hist         = roll(self.y_hist, -1)
            self.y_hist[-1]     = y_new
            self.y_t            = self.y_t + (1/self.n)*(y_new - self.y_hist[0])
        else:
            self.y_hist         = roll(self.y_hist, -1)
            self.y_hist[:,-1]   = y_new
            self.y_t            = self.y_t + (1/self.n)*(y_new - self.y_hist[:,0])
            
    def wgt_moving_avg(self, y_new):
        self.y_hist         = roll(self.y_hist, -1, axis = 1)
        self.y_hist[-1]     = y_new
        self.y_t            = dot(self.y_hist, self.wgt) / self.wgt_sum
        
    def getSignal(self, src = 'flt'):
        if src == 'flt':
            return self.y_t
        elif src == 'raw':
            return self.y_t_RAW
        else:
            raise ValueError('Enter valid return signal')
            

def estimate_position(Y_data, X_estimate, dt):
    """
    function    : estimate_position
    sensor      : myAHRS+
    
    input       : sensor readings (acc_data, rot_data)
    output      : state estimate from IMU sensor for v_x and v_y
    
    assumptions: a_z = v_z = w_x = w_y = 0
    """
    #### unpack sensor data 
    (BF_data, GF_data)      = X_estimate
    (a_x, a_y, psi, w_z)    = Y_data.getSignal()
    (v_x, v_y)              = BF_data.getSignal('raw')
    X_t                     = GF_data.getSignal()
    
    # estimate filtered velocities   
    A = array([[1,          w_z*dt], 
               [-w_z*dt,    1]])
    X = array([v_x, v_y]) 
    B = dt*array([a_x, a_y])
    v_next          = dot(A,X) + B
    BF_data.update(list(v_next))
    
    # compute the position    
    A = array([[1,      0,      dt,     0],
               [0,      1,      0,      dt],
               [0,      0,      0,      0],
               [0,      0,      0,      0],])
    B = array([[0,0],
               [0,0],
               [1,0],
               [0,1]])
    R = array([[cos(psi),   sin(psi)],
               [-sin(psi),  cos(psi)]])
    u = dot(R, array([v_x,v_y]))
    X_next = dot(A, X_t) + dot(B,u)
    GF_data.update(list(X_next))
    
    
def compute_yaw_rate(psi_prev, psi_new, dt):
    
    if (psi_prev < -3 and psi_new > 3):
        psi_prev += 2*pi
    if (psi_prev > 3 and psi_new < -3):
        psi_prev -= 2*pi
        
    return (psi_new - psi_prev)/dt
