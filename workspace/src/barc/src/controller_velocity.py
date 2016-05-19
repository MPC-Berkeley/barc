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

# This controller can be launched with cruiseControl.launch
# Requires a kinematic state estimator node and an arduino hub node

import rospy
from barc.msg import ECU, Z_KinBkMdl
from geometry_msgs.msg import Vector3
from pid import PID

# Save the latest velocity from the state estimator
def state_estimate_callback(msg):
    global v_est
    v_est = msg.v

# Change the controller's reference velocity as commanded
# Currently do this from the command line with `rostopic pub -1 ...`
# but this could be used with another controller that publishes desired velocity
def v_ref_callback(msg):
    global pid
    pid.setPoint(msg.x)

#############################################################
def control_velocity():
    global pid, v_est

    rospy.init_node('controller_velocity')

    motor_pub = rospy.Publisher('motor_pwm', ECU, queue_size = 10)
    rospy.Subscriber('v_ref', Vector3, v_ref_callback, queue_size=10)
    rospy.Subscriber('state_estimate', Z_KinBkMdl, state_estimate_callback, queue_size=10)

    # set node rate
    rateHz  = 50
    dt      = 1.0 / rateHz
    rate    = rospy.Rate(rateHz)

    # initialize motor_pwm and v_est at neutral
    v_est = 0
    motor_pwm = 90

    # use pid to correct velocity, begins with set point at zero
    p       = rospy.get_param("controller/p")
    i       = rospy.get_param("controller/i")
    d       = rospy.get_param("controller/d")
    pid     = PID(P=p, I=i, D=d)

    # main loop
    while not rospy.is_shutdown():
        u = pid.update(v_est, dt)

        motor_pwm = motor_pwm + u

        # send command signal
        motor_cmd = ECU(motor_pwm, 0)
        motor_pub.publish(motor_cmd)

        rate.sleep()

#############################################################
if __name__ == '__main__':
    try:
        control_velocity()
    except rospy.ROSInterruptException:
        pass
