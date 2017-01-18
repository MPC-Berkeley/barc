#!/usr/bin/env python
import rospy
from barc.msg import ECU
from std.msgs.msg import String


def lead():
    sig = rospy.Publisher('signal', barc/ECU, queue_size=10)
    ecu = rospy.Publisher('/ecu', barc/ECU, queue_size=10)

    rospy.init_node('Car_Master')

    r = rospy.rate(1)

    while not rospy.is_shutdown():
        cmd = ECU(0, 30)
        ecu.publish(cmd)
        sig.publish(cmd)

        r.sleep()

        cmd = ECU(0, -30)
        ecu.publish(cmd)
        sig.publish(cmd)

        r.sleep()

if __name__ == '__main__':
    try:
        lead()
    except rospy.ROSInterruptException:
        pass
    



