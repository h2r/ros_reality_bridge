#!/usr/bin/env python

import rospy
import moveit_commander
from std_msgs.msg import String, Header
import moveit_msgs.msg
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from ros_reality_bridge.msg import MoveitTarget, MoveitPlan
from moveit_msgs.msg import RobotState, DisplayTrajectory
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
    group.set_pose_reference_frame('/base_link')
    self.get_prev_id = {} # id to prev id in sequence
    self.get_next_id = {} # id to next id in sequence
    # self.get_angles = {} # id to pose of the arm in said state
    self.get_pose = {} # id to end effector pose

    self.get_gripper_state = {} # id to 1/0 gripper open
    self.get_plan = {} # id to the motion plan that ends in getting to id pose
    self.first_movable_point_id = None # pointer to the first moveable point in the sequence of waypoints
    self.names = [] # names of the joints of the rob

    self.ik_fail_pub = rospy.Publisher('ros_reality_ik_status', String, queue_size=0)
    self.gripper_pub = rospy.Publisher("/ein/right/forth_commands", String, queue_size = 0)
    # self.plan_pub = rospy.Publisher("ros_reality_motion_plan", MoveitPlan, queue_size = 0)

  def send_plan_request(self, data):
    print self.get_pose
    # print data
    msg = data.id.data
    print msg
    if msg == "": # case where we are closing the client and need to erase the data associated w/ sesh
      print "RESET"
      self.initializer()
      return
    limb_joints = self.ik_solve_right(data.right_arm)
    
    # check that the pose is allowable
    if limb_joints == 0:
      print "fail"
      msg = msg + " FAIL"
      self.ik_fail_pub.publish(msg)
      return
    else:
      msg = msg + " SUCCESS"
      self.ik_fail_pub.publish(msg)

    # case where we are adding a point
    if data.id.data not in self.get_pose:
      self.network_point(data)
    else:
      self.get_gripper_state[data.id.data] = data.right_open.data
      self.get_pose[data.id.data] = data.right_arm

    waypoints = [] # should use a global priority queue for this ideally...
    #waypoints.append(self.group.get_current_pose().pose)
    if self.first_movable_point_id == None:
      return

    # create the motion plan
    curr_id = self.first_movable_point_id
    
    while curr_id != None:
      waypoints.append(copy.deepcopy(self.get_pose[curr_id]))
      curr_id = self.get_next_id[curr_id]
    self.group = moveit_commander.MoveGroupCommander('right_arm')
    self.group.set_planning_time(float(len(self.get_pose)))
    (plan, fraction) = self.group.compute_cartesian_path(waypoints, 0.01, 0.0)

    '''
    #planning part of the function
    if self.get_prev_id[data.id.data] == "START":
      self.group.set_start_state_to_current_state()
    else:
      self.set_start_pose(self.get_prev_id[data.id.data])

    #self.group = moveit_commander.MoveGroupCommander('right_arm')
    self.group.set_planning_time(2.0)
    (plan, fraction) = self.group.compute_cartesian_path([data.right_arm], 0.01, 0.0)
    self.publish_plan(plan, data.id)
    print plan
    if data.prev_id.data == "START":
      self.names = plan.joint_trajectory.joint_names # populate names field
    self.get_plan[data.id.data] = plan

    # grab the last point of the plan
    self.get_angles[data.id.data] = plan.joint_trajectory.points[-1].positions

    if self.get_next_id[data.id.data] != None: # case where there is a point after to move
      self.set_start_pose(data.id.data)
      self.group.set_planning_time(2.0)
      (plan, fraction) = self.group.compute_cartesian_path([self.get_pose[self.get_next_id[data.id.data]]], 0.01, 0.0)
      self.publish_plan(plan, data.id)
      self.get_plan[self.get_next_id[data.id.data]] = plan
    '''

  def move_to_goal(self, data):
    print "hi!!"
    curr = self.first_movable_point_id
    while curr != None:
      self.group.set_start_state_to_current_state()
      self.group.set_planning_time(2.0)
      (plan, fraction) = self.group.compute_cartesian_path([self.get_pose[curr]], 0.01, 0.0)
      self.group.execute(plan)
      print self.get_gripper_state[curr]
      if self.get_gripper_state[curr] == "1":
        self.gripper_pub.publish('openGripper')
      else:
        self.gripper_pub.publish('closeGripper')
      rospy.sleep(0.8)
      curr = self.get_next_id[curr]
    
  # def set_start_pose(self, id_data):
  #   joint_state = JointState()
  #   joint_state.header = Header()
  #   joint_state.header.stamp = rospy.Time.now()
  #   assert len(self.names) != 0
  #   joint_state.name = self.names
  #   joint_state.position = self.get_angles[id_data]
  #   moveit_robot_state = RobotState()
  #   moveit_robot_state.joint_state = joint_state
  #   self.group.set_start_state(moveit_robot_state)

  def network_point(self, data):
    self.get_gripper_state[data.id.data] = data.right_open.data
    self.get_pose[data.id.data] = data.right_arm
    if data.prev_id.data != "":
      if data.prev_id.data == "START":
        self.get_prev_id[data.id.data] = "START"
        self.first_movable_point_id = data.id.data
      else:
        self.get_next_id[data.prev_id.data] = data.id.data
        self.get_prev_id[data.id.data] = data.prev_id.data

    if data.next_id.data != "":
      self.get_prev_id[data.next_id.data] = data.id.data
      self.get_next_id[data.id.data] = data.next_id.data
    else:
      self.get_next_id[data.id.data] = None

  # def publish_plan(self, plan, end_id):
  #   msg = MoveitPlan()
  #   msg.id = end_id
  #   msg.plan = plan
  #   self.plan_pub.publish(msg)

  # def print_data(self, data):
  #   print data

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
    handler = PlanHandler()
    handler.start()
  except rospy.ROSInterruptException:
    moveit_commander.roscpp_shutdown()