#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import tf
import time


def message_builder(link_dict):
    """
    Builds the message to send on the ROS network to the VR computer

    Returns:
        msg (string): the message
    """
    msg = ""
    for k, v in link_dict.iteritems():
        trans, rot = v
        trans = [float("{0:.3f}".format(n)) for n in trans]
        rot = [float("{0:.3f}".format(n)) for n in rot]
        msg += '{}:{}^{};'.format(k, trans, rot)
    # remove spaces to save on space
    return msg.replace(' ', '')


def get_transform(link, tf_listener):
    """
    update the link_dict with the position and rotation (transform) of the link relative to the base of the robot

    Params:
        link (string): name of the link to get transform of
    """
    #tf_listener.waitForTransform("map",link,rospy.Time(),rospy.Duration(4.0))
    try:
        t = tf_listener.getLatestCommonTime("map", link)
        #t = rospy.Time.now()
        #tf_listener.waitForTransform("map",link,t,rospy.Duration(4.0))
        (trans, rot) = tf_listener.lookupTransform('map', link, t)
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException, tf.Exception) as e:
        print e
        return
    return trans, rot


def main():
    # initialize the ROS node
    rospy.init_node("unityNode", anonymous=False)

    # set up the publisher
    pub = rospy.Publisher("ros_unity", String, queue_size=0)
    # set up the tf listener
    tf_listener = tf.TransformListener()

    # create a rate to sleep after every loop
    rate = rospy.Rate(60)

    trans = (0, 0, 0)
    rot = (0, 0, 0, 0)

    # sleep to give time for publishers and listener to initilize
    rospy.Rate(1).sleep()

    # dictionary to store the position and rotation of each link in the robot
    link_dict = dict()

    # initialize keys in linkDict
    for link in tf_listener.getFrameStrings():
        if 'reference' not in link:
            link_dict[link] = (trans, rot)
    time.sleep(1)
    print "starting"

    # main loop. Updates the values for each link in link_dict and publish the values
    while not rospy.is_shutdown():
        for link in link_dict:
            link_dict[link] = get_transform(link, tf_listener)
        pub_string = message_builder(link_dict)
        pub.publish(pub_string)
        rate.sleep()

if __name__ == '__main__':
    main()
