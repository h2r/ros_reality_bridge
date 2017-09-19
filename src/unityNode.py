#!/usr/bin/env python

import rospy
import roslib

from std_msgs.msg import String
from baxter_core_msgs.msg import EndEffectorState
from baxter_core_msgs.msg import JointCommand
from sensor_msgs.msg import JointState
from sensor_msgs.msg import Image
import tf

class unityNode:
	def __init__(self):
		rospy.init_node("unityNode",anonymous=False)

		self.pub = rospy.Publisher("ros_unity",String,queue_size=10)
		self.rate = rospy.Rate(60)

		self.lGripper = rospy.Subscriber("/robot/end_effector/left_gripper/state",EndEffectorState,self.lGripper_callback)
		self.rGripper = rospy.Subscriber("/robot/end_effector/right_gripper/state",EndEffectorState,self.rGripper_callback)
		self.tfListener = tf.TransformListener()
		self.lGripperS = 0 
		self.rGripperS = 0

		trans = (0,0,0)
		rot = (0,0,0,0)

		rospy.Rate(1).sleep()

		self.linkDict = dict()
		for link in self.tfListener.getFrameStrings():
			if 'reference' not in link:
				self.linkDict[link] = (trans, rot, True)

		while not rospy.is_shutdown():
			for link in self.linkDict:
				self.getTransform(link)
			pubString = self.messageBuilder()
			self.pub.publish(pubString)
			self.rate.sleep()

	def messageBuilder(self):
		msg = ""
		msg += "left_gripper_grip:"+str(self.lGripperS)+";"
		msg += "right_gripper_grip:"+str(self.rGripperS)
		for k,v in self.linkDict.iteritems():
			trans, rot, pub = v
			if pub:
				msg += ';' + k + ':' + str(trans) + str(rot)
		#print msg
		return msg


	def lGripper_callback(self,message):
		self.lGripperS = float(message.position)
	def rGripper_callback(self,message):
		self.rGripperS = float(message.position)

	def getTransform(self, link):
		if self.tfListener.frameExists("/base") and self.tfListener.frameExists(link):
			try:
				t = self.tfListener.getLatestCommonTime("/base", link)
				(trans,rot) = self.tfListener.lookupTransform('/base', link, t)
			except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException, tf.Exception):
				return
			old_trans, old_rot, _ = self.linkDict[link]
			self.linkDict[link] = (trans, rot, True)

uN = unityNode()
