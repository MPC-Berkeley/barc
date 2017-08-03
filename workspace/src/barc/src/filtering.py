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

from numpy import array, dot, zeros, roll

class filteredSignal:
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
        
    def getFilteredSignal(self):
        return self.y_t
    
    def getRawSignal(self):
        return self.y_t_RAW
