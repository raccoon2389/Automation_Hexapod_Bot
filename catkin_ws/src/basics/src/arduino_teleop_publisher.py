#!/usr/bin/env python

import rospy
from std_msgs.msg import String

import sys, select, os
import tty, termios


def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

if __name__ == "__main__":
    settings = termios.tcgetattr(sys.stdin)
    rospy.init_node('beep')
    pub = rospy.Publisher('ww', String, queue_size=1)
    rate = rospy.Rate(60)

    while not rospy.is_shutdown():
        key = getKey()    
        pub.publish(key)
        rate.sleep()
