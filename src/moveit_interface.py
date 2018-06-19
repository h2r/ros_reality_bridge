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

class PlanHandler(object):
  def __init__(self):
    self.initializer()

  def initializer(self):
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_python_interface', anonymous=True)
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    self.group = moveit_commander.MoveGroupCommander('right_arm')
    # group = moveit_commander.MoveGroupCommander('both_arms') # 'both_arms'
    self.get_prev_id = {}
    self.get_next_id = {}
    self.get_pose = {}
    self.get_gripper_state = {}
    self.first_movable_point_id = None
    self.ik_fail_pub = rospy.Publisher('ros_reality_ik_status', String, queue_size=0)

  def send_plan_request(self, data):
    print data
    msg = data.id.data
    if msg == "": # case where we are closing the client and need to erase the data associated w/ sesh
      self.initializer()
    limb_joints = self.ik_solve_right(data.right_arm)
    
    if limb_joints == 0:
      print "fail"
      msg = msg + " FAIL"
      self.ik_fail_pub.publish(msg)
      return
    else:
      msg = msg + " SUCCESS"
      self.ik_fail_pub.publish(msg)


    if data.id.data in self.get_pose: # case where we are moving a point
      self.get_pose[data.id.data] = data.right_arm
      self.get_gripper_state[data.id.data] = data.right_open.data

    else: # case where we are getting a new point
      if data.next_id.data != "" and data.prev_id.data != "": # case where the point is sandwiched
        self.get_prev_id[data.id.data] = data.prev_id.data
        self.get_next_id[data.prev_id.data] = data.id.data
        self.get_next_id[data.id.data] = data.next_id.data
        self.get_prev_id[data.next_id.data] = data.id.data
        self.get_pose[data.id.data] = data.right_arm
        self.get_gripper_state[data.id.data] = data.right_open.data

      elif data.prev_id.data != "": # case where you are at the end of the trajectory
        # glself.first_movable_point_id
        if data.prev_id.data == "START":
          self.first_movable_point_id = data.id.data
        self.get_prev_id[data.id.data] = data.prev_id.data
        self.get_next_
id[data.prev_id.data] = data.id.data
        self.get_next_id[data.id.data] = None
        self.get_pose[data.id.data] = data.right_arm
        self.get_gripper_state[data.id.data] = data.right_open.data

      else:
        assert False

    waypoints = []
    #waypoints.append(self.group.get_current_pose().pose)
    if self.first_movable_point_id == None:
      return

    # create the motion plan
    curr_id = self.first_movable_point_id
    
    while curr_id != None:
      waypoints.append(copy.deepcopy(self.get_pose[curr_id]))
      curr_id = self.get_next_id[curr_id]
    self.group = moveit_commander.MoveGroupCommander('right_arm')
    self.group.set_planning_time(2.0)
    print waypoints
    (plan, fraction) = self.group.compute_cartesian_path(waypoints, 0.01, 0.0)
    print plan


  def move_to_goal(self, data):
    # global master_plan
    # if master_plan != None:
    #   group.execute(master_plan)
    # else:
    #   return
    return
    
  def ik_solve_right(self, p):
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

  def start(self):
    rospy.Subscriber('/goal_pose', MoveitTarget, self.send_plan_request)
    rospy.Subscriber('/ros_reality/move_to_goal', String, self.move_to_goal)
    rospy.spin()

if __name__ == '__main__':
  try:
    #main()
    handler = PlanHandler()
    handler.start()
  except rospy.ROSInterruptException:
    moveit_commander.roscpp_shutdown()