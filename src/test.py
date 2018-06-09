#!/usr/bin/env python

import rospy
from ros_reality_bridge.msg import MoveitTarget

def callback(data):
	pass

rospy.init_node('test_node', anonymous=True)
rospy.Subscriber('/goal_pose', MoveitTarget, callback)
rospy.spin()
