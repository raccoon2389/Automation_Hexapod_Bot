#!/usr/bin/env python

import rospy
from std_msgs.msg import String


def callback(msg):
    print(msg)


rospy.init_node('message_subscriber')

sub = rospy.Subscriber('ww', String, callback)

rospy.spin()
