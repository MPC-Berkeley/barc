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

import rospy
import time
from barc.msg import ECU
from simulator.msg import Z_DynBkMdl, eZ_DynBkMdl
from numpy import sin, cos, tan, arctan, array, dot, pi
from numpy import sign, argmin, sqrt, zeros
from system_models_simulator import f_KinBkMdl, f_DynBkMdl, ef_KinBkMdl, ef_DynBkMdl
# input variables
d_f         = 0
FxR         = 0

# raw measurement variables
yaw_prev = 0
(roll, pitch, yaw, a_x, a_y, a_z, w_x, w_y, w_z) = zeros(9)

# from encoder
v_x_enc     = 0
t0          = time.time()
n_FL        = 0                     # counts in the front left tire
n_FR        = 0                     # counts in the front right tire
n_FL_prev   = 0
n_FR_prev   = 0
r_tire      = 0.04                  # radius from tire center to perimeter along magnets [m]
dx_qrt      = 2.0*pi*r_tire/4.0     # distance along quarter tire edge

# ecu command update
def ecu_callback(data):
    global FxR, d_f
    FxR         = data.motor        # input acceleration
    d_f         = data.servo        # input steering angle

# state estimation node
def vehicle_simulator():

    # initialize node
    rospy.init_node('vehicle_simulator', anonymous=True)

    # topic subscriptions / publications
    rospy.Subscriber('ecu', ECU, ecu_callback)
    state_pub   = rospy.Publisher('z_vhcl', Z_DynBkMdl, queue_size = 10)
    state_error_frame_pub   = rospy.Publisher('ez_vhcl', eZ_DynBkMdl, queue_size = 10)

        # get vehicle dimension parameters
    L_a = rospy.get_param("L_a")       # distance from CoG to front axel
    L_b = rospy.get_param("L_b")       # distance from CoG to rear axel
    m   = rospy.get_param("m")         # mass of vehicle
    I_z = rospy.get_param("I_z")       # moment of inertia about z-axis
    vhMdl   = (L_a, L_b, m, I_z)

    # get tire model
    B     = rospy.get_param("tire_model/B")
    C     = rospy.get_param("tire_model/C")
    mu    = rospy.get_param("tire_model/mu")
    trMdl = ([B,C,mu],[B,C,mu])

    # get external force model
    a0    = rospy.get_param("air_drag_coeff")
    Ff    = rospy.get_param("friction")
    F_ext = (a0, Ff)

    # set node rate
    loop_rate   = 50
    dt          = 1.0 / loop_rate
    rate        = rospy.Rate(loop_rate)
    t0          = time.time()

    # set initial conditions 
    x   = 0
    y   = 0
    psi = 0
    v_x = 0
    v_y = 0
    r   = 0

    s    = 0
    ey   = 0
    epsi = 0
    while not rospy.is_shutdown():

        # publish state estimate
        bta         = arctan( L_a / (L_a + L_b) * tan(d_f) )

        if abs(v_x) > 0.05:
            z  = (x,  y,  psi, v_x, v_y, r)
            ze = (s, ey, epsi, v_x, v_y, r)
            u = (d_f, FxR)

            (x, y,   psi, v_x, v_y, r) = f_DynBkMdl(z, u, vhMdl, trMdl, F_ext, dt)
            (s, ey, epsi, v_x, v_y, r) = ef_DynBkMdl(ze, u, vhMdl, trMdl, F_ext, dt)

        else:
            z  = (x, y,   psi, v_x)
            ze = (s, ey, epsi, v_x)
            u = (d_f, FxR)

            (x, y,   psi, v_x) = f_KinBkMdl(z,u, (L_a, L_b), F_ext, dt)
            (s, ey, epsi, v_x) = ef_KinBkMdl(ze,u, (L_a, L_b), F_ext, dt)
            v_y     = 0
            r = 0
        
        # publish information
        state_pub.publish( Z_DynBkMdl(x, y, psi, v_x, v_y, r) )

        state_error_frame_pub.publish( eZ_DynBkMdl(s, ey, epsi, v_x, v_y, r) )
        
        # wait
        rate.sleep()

if __name__ == '__main__':
    try:
       vehicle_simulator()
    except rospy.ROSInterruptException:
        pass
