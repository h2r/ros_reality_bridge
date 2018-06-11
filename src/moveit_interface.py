#!/usr/bin/env python

import rospy
import moveit_commander
from std_msgs.msg import String
import moveit_msgs.msg
import geometry_msgs.msg
from ros_reality_bridge.msg import MoveitTarget
from moveit_msgs.msg import RobotState
from sensor_msgs.msg import JointState
import sys

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_python_interface', anonymous=True)
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group = moveit_commander.MoveGroupCommander('right_arm')
# group = moveit_commander.MoveGroupCommander('both_arms') # 'both_arms'

get_prev_id = {}
get_next_id = {}
get_pose = {}
first_movable_point_id = None
# plans = {}


def send_plan_request(data):
  print data

  if data.id in get_pose: # case where we are moving a point
    get_pose[data.id] = data.right_arm

  else: # case where we are getting a new point
    if data.next_id != "" and data.prev_id != "": # case where the point is sandwiched
      get_prev_id[data.id] = data.prev_id
      get_next_id[data.prev_id] = data.id
      get_next_id[data.id] = data.next_id
      get_prev_id[data.next_id] = data.id
      get_pose[data.id] = data.right_arm


    # elif data.next_id != "": # has next but not prev (so start point)
    #   # should only happen once
    #   start_id = data.id
    #   get_pose[data.id] = data.right_arm
    #   get_prev_id[data.id] = None
    #   if data.next_id not in get_pose:
    #     return # case where we only have one point in the map, which is the start point

    elif data.prev_id != "": # case where you are at the end of the trajectory
      if data.prev_id == "START":
        first_movable_point_id = data.id
      get_prev_id[data.id] = data.prev_id
      get_next_id[data.prev_id] = data.id
      get_next_id[data.id] = None
      get_pose[data.id] = data.right_arm

    else:
      assert False

  waypoints = []
  if first_movable_point_id == None:
    return

  # create the motion plan
  curr_id = first_movable_point_id
  
  while curr_id != None:
    waypoints.append(copy.deepcopy(get_pose[curr_id]))
    curr_id = get_next_id[curr_id]

  (plan, fraction) = group.compute_cartesian_path(waypoints, 0.01, 0.0)


def move_to_goal(data):
  group.go(wait=True)


def main():
  #rospy.Subscriber('/ros_reality/goal_pose', geometry_msgs.msg.Pose, send_plan_request)
  rospy.Subscriber('/goal_pose', MoveitTarget, send_plan_request)
  rospy.Subscriber('/ros_reality/move_to_goal', String, move_to_goal)
  rospy.spin()



if __name__ == '__main__':
  try:
    main()
  except rospy.ROSInterruptException:
    moveit_commander.roscpp_shutdown()