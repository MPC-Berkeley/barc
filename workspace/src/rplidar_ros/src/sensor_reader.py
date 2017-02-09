#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from rplidar_ros.msg import SensorReport
import numpy as np

PERIOD = 2
def listener():
    global last_pos, last_vel, count, scan_time
    count, scan_time = 1, 0
    last_vel, last_pos = np.zeros(2), np.zeros(2)
    rospy.init_node('lidar_monitor', anonymous=True)
    pub = rospy.Publisher('rplidar_msg', SensorReport, queue_size=10)

    def callback(msg):
        global last_pos, last_vel, count, scan_time
        if count < PERIOD:
            count += 1
            return
        stamp = msg.header.stamp
        time = round(stamp.secs%1e4 + stamp.nsecs*1e-9, 4)
        scan_range = msg.ranges[-90:] + msg.ranges[:270]
        min_value = np.min(scan_range)
        degree = np.argmin(scan_range) * np.pi / 180.
        dt = time - scan_time
        pos = np.array([np.cos(degree), np.sin(degree)]) * min_value
        vel = np.round((pos - last_pos)/dt, 4)
        last_pos, last_vel = pos, vel
        scan_time, count = time, 1
        print "position: ({}, {}), with velocity: ({}, {})".format(pos[0], pos[1], vel[0], vel[1])
        sr = SensorReport()
        sr.posX = pos[0]
        sr.posY = pos[1]
        sr.velX = vel[0]
        sr.velY = vel[1]
        pub.publish(sr)

    rospy.Subscriber("scan", LaserScan, callback)


    rospy.spin()


if __name__ == '__main__':
    listener()
