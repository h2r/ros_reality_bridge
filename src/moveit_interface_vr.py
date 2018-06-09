#!/usr/bin/env python

import rospy
import moveit_commander
from std_msgs.msg import String
import moveit_msgs.msg
import geometry_msgs.msg
from ros_reality_bridge.msg import MoveitTarget
import sys

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_python_interface', anonymous=True)
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group_left = moveit_commander.MoveGroupCommander('left_arm') # 'both_arms'
group_right = moveit_commander.MoveGroupCommander('right_arm') # 'both_arms'

def send_plan_request_left(data):
  print data
  #group.set_pose_target(data)
  
  group_left.set_pose_target(data, 'left_gripper')
  group_left.set_planning_time(1.0)
  plan = group_left.plan()

def send_plan_request_right(data):
  print data
  #group.set_pose_target(data)
  
  group_right.set_pose_target(data, 'right_gripper')
  group_right.set_planning_time(1.0)
  plan = group_right.plan()

def move_to_goal_left(data):
  group_left.go(wait=True)

def move_to_goal_right(data):
  group_right.go(wait=True)

def main():
  #rospy.Subscriber('/ros_reality/goal_pose', geometry_msgs.msg.Pose, send_plan_request)
  rospy.Subscriber('/goal_pose_left', geometry_msgs.msg.Pose, send_plan_request_left)
  rospy.Subscriber('/goal_pose_right', geometry_msgs.msg.Pose, send_plan_request_right)
  rospy.Subscriber('/move_to_goal_left', String, move_to_goal_left)
  rospy.Subscriber('/move_to_goal_right', String, move_to_goal_right)
  rospy.spin()



if __name__ == '__main__':
  try:
    main()
  except rospy.ROSInterruptException:
    moveit_commander.roscpp_shutdown()