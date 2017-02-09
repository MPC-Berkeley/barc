import tf
import rospy
import sys
import math
import numpy as np
from tf2_msgs.msg import TFMessage
from perception.msg import Status
import exp_quat_func as eqf

lkp = {} # dictionary containing the last known position of AR tags
lkp_old = {}
lkp_extre_old = {}
SPEED_CONST = 1
def callback(msg, argument):
    ar_tags, publisher = argument
    # rate = rospy.Rate(10)
    
    # print "getting mesage infos..."
    global lkp, lkp_old, lkp_extre_old
    lkp_extre_old, lkp_old = lkp_old.copy(), lkp.copy()
    
    for i in range(0, len(msg.transforms)):
        stamp = msg.transforms[i].header.stamp
        time = round(stamp.secs%1e4 + stamp.nsecs*1e-9, 4)
        lkp[msg.transforms[i].child_frame_id] = \
                ([msg.transforms[i].transform.translation.x, \
                  msg.transforms[i].transform.translation.y, \
                  msg.transforms[i].transform.translation.z], \
                 [msg.transforms[i].transform.rotation.x, \
                  msg.transforms[i].transform.rotation.y, \
                  msg.transforms[i].transform.rotation.z, \
                  msg.transforms[i].transform.rotation.w], \
                  time) # position / orientation
    # print "computing twist and positions"

    for ar in ar_tags.values():
        try:
            ar_trans = np.array(lkp[ar][0])
            ar_trans_old = np.array(lkp_extre_old[ar][0])

            ar_quat = np.array(lkp[ar][1])
            dpos = ar_trans - ar_trans_old
            # dtime = lkp[ar][2] - lkp_extre_old[ar][2]
            omega, theta = eqf.quaternion_to_exp(ar_quat)
            v = eqf.find_v(omega, theta, ar_trans)

            cmd = Status()
            cmd.px, cmd.py, cmd.pz = ar_trans
            cmd.vx, cmd.vy, cmd.vz = dpos*10
            cmd.wx, cmd.wy, cmd.wz = np.zeros((3,))
            print np.round(dpos*10, 3)
            # cmd.vx = v[0] * SPEED_CONST
            # cmd.vy = v[1] * SPEED_CONST
            # cmd.vz = v[2] * SPEED_CONST
            # cmd.wx = omega[0] * theta * SPEED_CONST
            # cmd.wy = omega[1] * theta * SPEED_CONST
            # cmd.wz = omega[2] * theta * SPEED_CONST
            publisher.publish(cmd)
        except:
            continue


if __name__=='__main__':
    rospy.init_node('detection', anonymous=True)

    if len(sys.argv) < 2:
        print('Usage: detection.py [AR_Tags]')
        sys.exit()

    ar_tags = {}
    for tag in sys.argv[1:]:
        ar_tags['ar'+tag] = 'ar_marker_' + tag

    # print "reading tages....."
    # print ar_tags
    # print "setup publisher...."
    publisher = rospy.Publisher('user_messages', Status, queue_size=2)
    rate = rospy.Rate(10)

    # print "trying to subscribe from tf..."
    # while not rospy.is_shutdown():
    rospy.Subscriber('/tf', TFMessage, callback, (ar_tags, publisher))
    
    rospy.spin()
