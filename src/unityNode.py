import rospy
import roslib
import moveit_msgs.msg
#from ein.msg import EinState
from std_msgs.msg import String
from baxter_core_msgs.msg import EndEffectorState
from baxter_core_msgs.msg import JointCommand
from sensor_msgs.msg import JointState
from sensor_msgs.msg import Image


class unityNode:
	def __init__(self):
		rospy.init_node("unityNode",anonymous=False)

		self.pub = rospy.Publisher("ros_unity",String,queue_size=10)
		self.rate = rospy.Rate(120)

		self.lGripper = rospy.Subscriber("/robot/end_effector/left_gripper/state",EndEffectorState,self.lGripper_callback)
		self.rGripper = rospy.Subscriber("/robot/end_effector/right_gripper/state",EndEffectorState,self.rGripper_callback)
		self.lJoints = rospy.Subscriber("/robot/limb/left/joint_command",JointCommand,self.lJoints_callback)
		self.rJoints = rospy.Subscriber("/robot/limb/right/joint_command",JointCommand,self.rJoints_callback)
		self.joint_angles_sub = rospy.Subscriber("/robot/joint_states",JointState,self.joint_angles_callback)

		#Real Robot state	
		#self.jointTracjectory = rospy.Subscriber("/move_group/display_planned_path",moveit_msgs.msg.DisplayTrajectory,self.jointTrajectory_callback)
		self.lGripperS = 0 
		self.rGripperS = 0
		self.left_arm_angles = [0,0,0,0,0,0,0]
		self.right_arm_angles = [0,0,0,0,0,0,0]
		self.head_angles = [0,0]
		self.torso = 0

		while not rospy.is_shutdown():
			pubString = self.messageBuilder()
			#pubString = "Cats"
			self.pub.publish(pubString)
			self.rate.sleep()

	def messageBuilder(self):
		msg = ""
		msg += "left_gripper:"+str(self.lGripperS)+","
		msg += "right_gripper:"+str(self.rGripperS)+","
		msg += "head_nod:"+str(self.head_angles[0])+","
		msg += "head_pan:"+str(self.head_angles[1])+","
		for l_joint in range(len(self.left_arm_angles)):
			msg += "l_"+str(l_joint)+":"+str(self.left_arm_angles[l_joint])+","
		for r_joint in range(len(self.right_arm_angles)):
			msg += "r_"+str(r_joint)+":"+str(self.right_arm_angles[r_joint])+","
		msg += "torso:"+str(self.torso)
		return msg

		
	def XMLSetup(self):
		#preString = "<textarea rows=\"20\" cols=\"40\" style=\"border:none;\">"
		#xmlString = preString + "<rosUnity>"
		xmlString = "<rosUnity>"
		xmlString += "<left_gripper>" + str(self.lGripperS) + "</left_gripper>"
		xmlString += "<right_gripper>" + str(self.rGripperS) + "</right_gripper>"
		xmlString += "<head_nod>" + str(self.head_angles[0]) + "</head_nod>"
		xmlString += "<head_pan>" + str(self.head_angles[1]) + "</head_pan>"
		for l_joint in range(len(self.left_arm_angles)):
			xmlString += "<l_"+str(l_joint)+">"+str(self.left_arm_angles[l_joint])+"</l_"+str(l_joint)+">"
		for r_joint in range(len(self.right_arm_angles)):
			xmlString += "<r_"+str(r_joint)+">"+str(self.right_arm_angles[r_joint])+"</r_"+str(r_joint)+">"
		xmlString += "<torso>" + str(self.torso) + "</torso>"
		xmlString += "</rosUnity>" #+ "</textarea>"	
		#print xmlString
		return xmlString
	def lGripper_callback(self,message):
		self.lGripperS = float(message.position)
		#print self.lGripperS
	def rGripper_callback(self,message):
		self.rGripperS = float(message.position)
		#print self.rGripperS
	def lJoints_callback(self,message):
		self.lJointsS = message.command
		#print len(message.command)
	def rJoints_callback(self,message):
		self.rJointsS = message.command
		#print message
	def jointTrajectory_callback(self,message):
		#print message.trajectory[0].joint_trajectory.points
		pass
	def joint_angles_callback(self,message):
		angles = message.position
		self.head_angles[0] = angles[0]
		self.head_angles[1] = angles[1]
		for l_joint in range(2,9):
			self.left_arm_angles[l_joint-2] = angles[l_joint]
		for r_joint in range(9,16):
			self.right_arm_angles[r_joint-9] = angles[r_joint]
		self.torso = angles[16]
		

uN = unityNode()
