Source code for LMPC
====================

1. barc_record.jl
=================
This node records data that might be interesting for later evaluation. It subscribes to topics, records received messages into arrays and saves these arrays into a .jld file when the ros master is shut down. It uses log_functions.jl in barc_lib to import the callback functions.

2. barc_simulator_dyn.jl
========================
This node simulates a dynamic bicycle model. It subscribes to the ECU commands of the MPC and publishes simulated "raw" measurement data: GPS, IMU and Encoders. It also logs its "real" states into a .jld file, this can be interesting to experiment with Kalman filters to find a good estimator (since you can compare "real" simulation data with your estimation data).
It uses the simModel.jl in barc_lib.

3. controller_low_level.py
==========================
This is the low level controller that maps acceleration and steering angle commands (in SI units) to PWM signals that are needed by the motor and steering servo. The mapping is linear and has been found by multiple open-loop measurements. It might vary from BARC to BARC.

4. controller_MPC_traj.jl
=========================
This is a simple MPC path following controller that uses a given track.

5. debug_localization.py
========================
This is a very short script you can use to debug and analyze the mapping from x/y coordinates to s/e_y/e_psi. You can use it to find an optimal approximation length for the curvature.

6. filtering.py
===============
Basic filtering classes, not used in LMPC but might be helpful for future use.

7. LMPC_node.jl
===============
This is the main node of LMPC. It receives data from the estimator, records it and does system ID and LMPC to find optimal inputs.

8. Localization_helpers.py
==========================
This contains a class that is used to map x-y-coordinates to s-ey-epsi coordinates. It also includes the shape of the track (in create_track()). Tracks are created by putting together curves using the function add_curve(track,length,angle_of_curve). This makes sure that there are no jumps in curvature (but there are jumps in the 1st derivative of the curvature!).
Mapping x-y to s-ey:
1. set_pos() sets the class variables to the current position/heading
2. find_s() calculates s, ey, epsi and the curvature coefficients according to its settings (polynomial degree, length of approximation)
3. read variables (s,ey, ...) manually from class (look at state estimation node to get an idea)

9. observers.py
===============
Contains the function for the EKF and other necessary functions.

10. open_loop.jl
================
Can do open loop experiments with this and send either ECU or ECU_pwm commands.

11. state_estimation_SensorKinematicModel.py
============================================
State estimation node. Uses a combination of a kinematic bicycle model and a kinematic motion model in an EKF (without the bicycle assumption) to calculate states. It also prefilters GPS data to make sure holes and outliers are not used.

12. system_models.py
====================
This contains the model that is used by the Kalman filter in state estimation.

