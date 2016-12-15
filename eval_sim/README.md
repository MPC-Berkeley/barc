Overview over evaluation functions
==================================
All evaluation functions use a code (String) which specifies an experiment. It is a 4-character combination which is created by the ROS master. All files that are saved during one experiment use this code.
Files that are saved: "output-record-(code).jld" from the recorder, "output-LMPC-(code).jld" from the LMPC node, and, if it is a simulation, "output-SIM-(code).jld" (contains real state information).
!! at the beginning denots most used functions.

eval_data.jl
============
Main file for all kinds of post-experiment evaluation

eval_sim(code)
--------------
Used for general evaluation of *simulations*. It uses 'real' simulation and 'measured' data and compares these. This can be helpful to develop a good state estimator.

!! eval_run(code)
-----------------
Most general evaluation of experiments. Similar to eval_sim, but does not use simulation data.

!! eval_LMPC(code)
------------------
More detailed evaluation of specific LMPC data (MPC cost, sysID coefficients, curvature approximations, ...)

plot_friction_circle(code, lap)
-------------------------------
Plots a_y over a_x.

plot_v_ey_over_s(code, laps)
----------------------------
Plots v and e_y over s for selected laps. Also plots lap times for all laps.

plot_v_over_xy(code, lap)
-------------------------
Plots velocity in different colors on xy.

eval_open_loop(code)
--------------------
Similar to eval_run but for open-loop experiments (which are saved in a different folder).

eval_predictions_kin(code)
-------------------------
Plots predictions for kinematic model (in path following mode)

eval_predictions(code)
---------------------
Plots predictions for dynamic model (in LMPC mode)

eval_sysID(code)
---------------
Plots sysID coefficients

eval_oldTraj(code, lap)
-----------------
Plots data of one selected oldTrajectory (-> data of one specific lap +overlapping data before and after finish line)

eval_LMPC_coeff(code, step)
--------------------
Plots terminal constraints (both polynom and prediction) for one specific step in the LMPC process.

!! anim_LMPC_coeff(code)
--------------------
Animates predictions and terminal constraints (starting from first LMPC step)

checkTimes(code)
----------------
Check solving times and status (optimal/infeasible/user limit).

Helper functions:
----------------
create_track(half_track_width)  -> used to create the track for plotting
initPlot()  -> used to initialize plots for proper export and printing

sim_kalman_imu.jl
=================
This file simulates a Kalman filter using real sensor data that was recorded during an experiment. It can be used to find the optimal structure and tuning values for the estimator.

Parameter estimation folder
===========================
Contains various files and functions to estimate BARC specific input mapping parameters (e.g. plot steering delay, least-square approximation for acceleration and steering, ...)