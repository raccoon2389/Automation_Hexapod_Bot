#!/usr/bin/env python

import rospy
from std_msgs.msg import String

rospy.init_node('beep')
pub = rospy.Publisher('ww', String, queue_size=1)
rate = rospy.Rate(1)

key = 'w'

while not rospy.is_shutdown():    
    pub.publish(key)
    rate.sleep()

