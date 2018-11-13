#!/usr/bin/env python
import getch
import rospy
from std_msgs.msg import Float32

'''
    File name: keyboard_controller.py
    Author: Rohan Sinha
    Email: rohan.sinha@berkeley.edu
    Python Version: 2.7.12
'''

def set_offset(char, offset): 
    if (offset <= .9) and (char == 'a'):
        return offset + 0.1
    elif (offset >= -.9) and (char == 'd'):
        return offset - .1
    else: 
        return offset

def key_node():
    rospy.init_node('key_node')
    pub = rospy.Publisher('key_input', Float32)
    rospy.init_node('key_node')
    print('Use A to increase offset by 10 cm, use D to decrease offset by 10 cm')
    print('Use F to exit')
    offset = 0
    while not rospy.is_shutdown():
        msg = Float32()
        msg.data = offset
        pub.publish(msg)
        char = getch.getch()
        offset = set_offset(char, offset)
        if char == 'f':
            exit(0)

if __name__=="__main__":
    try:
        key_node()
    except rospy.ROSInterruptException:
        pass