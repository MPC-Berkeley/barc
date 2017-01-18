#!/usr/bin/env python

import rospy
from barc.msg import ECU
from std.msgs.msg import String

class Proxy():

    def __init__(self):
        self.ecu = rospy.Publisher('/ecu', barc/ECU, queue_size=10)
        rospy.init_node('Car_Slave')

    def callback(data):
        self.ecu.publish(data)


    def follow():

        rospy.Subscriber('signal', barc/ECU, callback)

        rospy.spin()

}

if __name__ == '__main__':
    car = Proxy()
    car.follow()

