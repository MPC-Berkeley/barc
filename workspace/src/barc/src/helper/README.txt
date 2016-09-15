This file helps to explain the structure of the LMPC control.


1. The estimator and Localization node
======================================
The estimator estimates the current state (x, y, psi, v) by using a Kalman filter on the sensor data.
The localization node maps this estimated data on the given track and calculates following values:
s_start = distance along the track, starting from the start/finish line to where a polynomial approximation of the track's curvature starts
s       = distance along the track, starting at s_start, to the current position of the car
eY 	    = distance of the car to the track (perpendicular to track)
ePsi    = difference of the car's heading and the current reference heading (tangent to the track)
v       = current velocity
The track's shape has to be defined in Localization_node.py.

2. The LMPC node
================
The LMPC node receives the position info (s_start, s, eY, ePsi, v) and first calculates the mpc coefficients (they are part of the "learning" part and account for previous laps). The coefficients approximate polynomials that are used in the MPC control, in the interval closest_s to closest_s + 2*N.
These coefficients are then sent to the solveMpcProblem.jl function which calculates the new optimal input.


Further info
============
Confusion with different s's:
s_target = length of the track

Localization node:
------------------
It returns s_start in the interval of 0 < s_start < s_target and s in the interval 0 < s < nPoints*ds (with nPoints = number of nodes along the track which are used to calculate the approximate polynomial and ds = distance between nodes).
s, s_start and the polynomial approximation coefficients are received by the LMPC_node and written in the state *and* posInfo.
Important: The polynomial coefficients are only valid in the interval 0 < s < nPoints*ds, starting from s_start !!!

coeffConstraintCost:
--------------------
This function calculates coefficients which approximate the previous trajectory around the current position s_total = s_start + s. The approximated polynomials are valid in the range s_total < s_poly < s_total + 2*N (with N = prediction horizon of the MPC).

solveMpcProblem:
----------------
This is the MPC solver, it calculates the optimal input controls by predicting the state evolution, starting at the current state. It uses the states [s, ey, epsi, v] and the coefficients from the Localization node (track coefficients) for calculating the evolution. Therefore, it is important that for the state evolution the relative value s is used!
However, there's a second part to the MPC, which calculates terminal constraints and cost (heart of the learning part).