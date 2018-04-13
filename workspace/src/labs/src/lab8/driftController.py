#!/usr/bin/env python

import rospy
from barc.msg import ECU, Encoder
from pid import PID
from observer import EncoderModel, ImuModel
from sensor_msgs.msg import Imu
from numpy.random import uniform
from numpy import around

def main():

    # initialize velocity estimator
    enc = EncoderModel()
    imu = ImuModel()

    # initialize node, set topic subscriptions / publications
    rospy.init_node('driftController', anonymous=True)
    rospy.Subscriber('/encoder', Encoder, enc.estimateVelocityM1)
    rospy.Subscriber('/imu/data', Imu, imu.updateEstimates)
    ecu_pub   = rospy.Publisher('/ecu_pwm', ECU, queue_size = 10)

    # initialize pid controller
    loop_rate   = rospy.get_param("controller/loop_rate")
    v_ref       = rospy.get_param("controller/v_ref")
    Pm          = rospy.get_param("controller/Pm")
    Im          = rospy.get_param("controller/Im")
    Dm          = rospy.get_param("controller/Dm")
    Ps          = rospy.get_param("controller/Ps")
    Is          = rospy.get_param("controller/Is")
    Ds          = rospy.get_param("controller/Ds")

    # pid controller
    rate        = rospy.Rate(loop_rate)
    dt          = 1.0/loop_rate
    pid_motor   = PID(Pm,Im,Dm)
    pid_servo   = PID(Ps,Is,Ds)
    pid_motor.setTimeStep(dt)
    pid_servo.setTimeStep(dt)
    pid_motor.setPoint(v_ref)
    pid_servo.setPoint(0.0)

    # sample input parameters for drift
    s               = 3.0
    Dt              = uniform(0,0.8)
    #F1              = int( uniform( a, b ) )
    #df              = int( uniform( a, b ) )
    F2              = 990   # brake

    u_motor_neutral = 1500
    u_servo_neutral = 1500
    u_motor         = u_motor_neutral
    u_servo         = u_servo_neutral

    rospy.logwarn("Dt = {}".format(Dt))
    #rospy.logwarn("F1 (acceleration) = {}".format(F1))
    #rospy.logwarn("df (steering) = {}".format(df))

    now         = rospy.get_rostime()
    t0          = now.secs + now.nsecs/(10.0**9)
    straight    = False        
    turn        = False
    brake       = False
 
    while not rospy.is_shutdown():
        # get time
        now = rospy.get_rostime()
        t   = now.secs + now.nsecs/(10.0**9) - t0
        
        # get vehicle into initial state
        if enc.s_m1 < s:
            if not straight:
                rospy.logwarn("Going straight ...")
                straight = True

             # compute feedforward / feedback command for motor
            u_ff    = u_motor_neutral
            u_fb    = pid_motor.update( enc.vhat_m1 )
            u_motor = u_ff + int(u_fb)
            
            # compute feedforward / feedback command for servo
            u_ff    = u_servo_neutral
            u_fb    = pid_servo.update( -imu.dy_deg )
            u_servo = u_ff + int(u_fb)

            t_straight  = t
        else:
            u_motor = u_motor_neutral
            u_servo = u_servo_neutral

        # perform aggresive turn and accelerate
        """
        elif t < t_straight + ???:
            if not turn:
                rospy.logwarn("Turning and accelerating ...")
                turn = True
            u_motor = ??? 
            u_servo = ???

        # apply brake
        else:
            if not brake:   
                rospy.logwarn("Braking ! ...")
                brake = True
            u_motor = ???
            u_servo = ???
        """

        # publish control command
        #rospy.logwarn("v1 = {}".format(enc.vhat_m1))
        #rospy.logwarn("s1 = {}".format(enc.s_m1))
        #rospy.logwarn("yaw = {}".format(imu.dy))
        ecu_pub.publish( ECU(u_motor, u_servo) )

        # wait
        rate.sleep()

if __name__ == '__main__':
    try:
       main()
    except rospy.ROSInterruptException:
        pass
