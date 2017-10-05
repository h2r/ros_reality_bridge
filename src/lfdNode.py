#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import tf
from ein.msg import EinState

def record_callback(data):
	print hi

def state_callback(data):
	print type(data)

def main():
    # initialize the ROS node
    print "hi"
    rospy.init_node("lfdNode", anonymous=False)

    # set up the subscriber
    rospy.Subscriber("/demonstrations", String, record_callback)
    rospy.Subscriber("/ein/right/state", EinState, state_callback)

    # create a rate to sleep after every loop
    rate = rospy.Rate(60)

    # sleep to give time for publishers and listener to initilize
    rospy.Rate(1).sleep()

    while not rospy.is_shutdown():
        rate.sleep()

if __name__ == '__main__':
    main()
