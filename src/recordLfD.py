#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import tf
from ein.msg import EinState

recordings = []

targ = []
true = []

def record_callback(data):
	global recordings
	recordings.append([targ,true])


def state_callback(data):
	global targ, true
	targ = data.state_string.split("\n")[1].split(":")[1].split(" ")[1:]
	true = data.state_string.split("\n")[3].split(":")[1].split(" ")[1:]

	targ = [float(i) for i in targ]
	true = [float(i) for i in true]


def main():
    global recordings
    # initialize the ROS node
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

    thefile = open('test.txt','w')
    for item in recordings:
	thefile.write("%s\n" % item)

if __name__ == '__main__':
    main()
