#!/usr/bin/env python

import rospy
from barc.msg import ECU, Z_KinBkMdl
from numpy import cos, sin, cos, tan, sqrt
from std_msgs.msg import Float32

class kinMdl:
    def __init__(self):
        self.z  = [0.0, 0.0, 0.0, 0.0]
        self.Ts = None
        self.L  = None
        self.u  = [0.0, 0.0]

    def updateInput(self, msg):
        acc     = msg.motor
        df      = msg.servo
        self.u  = [df, acc]


    def simulateOneTimeStep(self):
        # extract system states, input, and parameters
        (x,y,psi,v)  = self.z
        (df,acc)     = self.u
        (Ts, L)      = self.Ts, self.L

        # evolve system dynamics
        xNext   = x     + Ts*(v*cos( psi )) 
        yNext   = y     + Ts*(v*sin( psi )) 
        psiNext = psi   + Ts*(v*tan( df )/L) 
        vNext   = v     + Ts*(acc) 

        # update state
        z = [xNext, yNext, psiNext, vNext]
        self.z = z

        return z                
                           

# state estimation node
def main():

    # initialize node
    sysMdl = kinMdl()
    rospy.init_node('KinematicBikeModel', anonymous=True)

    # topic subscriptions / publications
    rospy.Subscriber('controller', ECU, sysMdl.updateInput)
    state       = rospy.Publisher('plant', Z_KinBkMdl, queue_size = 1)

    # get vehicle dimension parameters
    sysMdl.L    = rospy.get_param("/vehicle_length")  
    sysMdl.Ts   = rospy.get_param("/simulation_time_step")

    # get / set initial state
    sysMdl.z    = rospy.get_param("/initial_state")

    # set node rate
    loop_rate   = int( 1.0 / sysMdl.Ts )
    rate        = rospy.Rate(loop_rate)

    while not rospy.is_shutdown():

        # run simulation one time step
        (x, y, psi, v) = sysMdl.simulateOneTimeStep()

        # publish state information
        state.publish( Z_KinBkMdl(x, y, psi, v) )

        # wait
        rate.sleep()

if __name__ == '__main__':
    try:
       main()
    except rospy.ROSInterruptException:
        pass
