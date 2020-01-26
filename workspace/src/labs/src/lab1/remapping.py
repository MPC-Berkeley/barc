#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist

def callback_function(data):
    #FILL IN HERE
    global publisher_name, msg
    msg.linear.x = -data.linear.x
    msg.angular.z = -data.angular.z
    publisher_name.publish(msg)


def subscriber_name():
    # Initialize node
    rospy.init_node('subscriber_name', anonymous=True)

    #FILL IN HERE
    global publisher_name,msg
    msg = Twist()
    publisher_name = rospy.Publisher('remapped_topic_name',Twist,queue_size = 16)
    rospy.Subscriber('turtle1/cmd_vel', Twist, callback_function)
    rospy.Rate(30)
    rospy.spin()

if __name__ == '__main__':
    try:
        subscriber_name()
    except rospy.ROSInterruptException:
        pass
