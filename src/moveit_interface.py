#!/usr/bin/env python

import rospy
import moveit_commander
from std_msgs.msg import String, Header
import moveit_msgs.msg
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from ros_reality_bridge.msg import MoveitTarget
from moveit_msgs.msg import RobotState
from sensor_msgs.msg import JointState
import sys
import copy
from baxter_core_msgs.srv import SolvePositionIK, SolvePositionIKRequest
import baxter_interface

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_python_interface', anonymous=True)
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group = moveit_commander.MoveGroupCommander('right_arm')
# group = moveit_commander.MoveGroupCommander('both_arms') # 'both_arms'

get_prev_id = {}
get_next_id = {}
get_pose = {}
get_gripper_state = {}
first_movable_point_id = None
master_plan = None
# plans = {}


def send_plan_request(data):
  print data
  global get_prev_id 
  global get_next_id 
  global get_pose
  global get_gripper_state
  global first_movable_point_id
  global group
  global master_plan
  global ik_fail_pub


  limb_joints = ik_solve_right(data.right_arm)
  if limb_joints == 0:
    print "FAIL"
    ik_fail_pub.publish('f')
    return


  if data.id.data in get_pose: # case where we are moving a point
    get_pose[data.id.data] = data.right_arm
    get_gripper_state[data.id.data] = data.right_open.data

  else: # case where we are getting a new point
    if data.next_id.data != "" and data.prev_id.data != "": # case where the point is sandwiched
      get_prev_id[data.id.data] = data.prev_id.data
      get_next_id[data.prev_id.data] = data.id.data
      get_next_id[data.id.data] = data.next_id.data
      get_prev_id[data.next_id.data] = data.id.data
      get_pose[data.id.data] = data.right_arm
      get_gripper_state[data.id.data] = data.right_open.data


    # elif data.next_id != "": # has next but not prev (so start point)
    #   # should only happen once
    #   start_id = data.id
    #   get_pose[data.id] = data.right_arm
    #   get_prev_id[data.id] = None
    #   if data.next_id not in get_pose:
    #     return # case where we only have one point in the map, which is the start point

    elif data.prev_id.data != "": # case where you are at the end of the trajectory
      # global first_movable_point_id
      if data.prev_id.data == "START":
        first_movable_point_id = data.id.data
      get_prev_id[data.id.data] = data.prev_id.data
      get_next_id[data.prev_id.data] = data.id.data
      get_next_id[data.id.data] = None
      get_pose[data.id.data] = data.right_arm
      get_gripper_state[data.id.data] = data.right_open.data

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
  group = moveit_commander.MoveGroupCommander('right_arm')
  group.set_planning_time(5.0)
  (plan, fraction) = group.compute_cartesian_path(waypoints, 0.01, 0.0)
  print plan


def move_to_goal(data):
  # global master_plan
  # if master_plan != None:
  #   group.execute(master_plan)
  # else:
  #   return
  return
  
def ik_solve_right(p):
  ns = "ExternalTools/" + "right" + "/PositionKinematicsNode/IKService"
  iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
  ikreq = SolvePositionIKRequest()

  hdr = Header(stamp=rospy.Time.now(), frame_id='base')
  poses = {"right": PoseStamped(header=hdr, pose=p)}
  ikreq.pose_stamp.append(poses["right"])
  try:
    #rospy.wait_for_service(ns, 0.5)
    resp = iksvc(ikreq)
  except (rospy.ServiceException, rospy.ROSException), e:
    rospy.logerr("Service call failed: %s" % (e,))
    return 0
  if (resp.isValid[0]):
    # Format solution into Limb API-compatible dictionary
    limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
    return limb_joints
  else:
    return 0


def main():
  #rospy.Subscriber('/ros_reality/goal_pose', geometry_msgs.msg.Pose, send_plan_request)
  #global get_prev_id, get_next_id, get_pose, get_gripper_state, first_movable_point_id, group, master_plan, ik_fail_pub
  ik_fail_pub = rospy.Publisher('ros_reality_ik_status', String, queue_size=0)
  rospy.Subscriber('/goal_pose', MoveitTarget, send_plan_request)
  rospy.Subscriber('/ros_reality/move_to_goal', String, move_to_goal)
  rospy.spin()



if __name__ == '__main__':
  try:
    main()
  except rospy.ROSInterruptException:
    moveit_commander.roscpp_shutdown()