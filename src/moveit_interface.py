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
# from baxter_core_msgs.msg import EndEffectorState
import baxter_interface
import math

class PlanHandler(object):
  def __init__(self):
    self.initializer()

  def initializer(self):
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_python_interface', anonymous=True)
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()

    self.group = moveit_commander.MoveGroupCommander('right_arm')
    self.get_prev_id = {} # id to prev id in sequence
    self.get_next_id = {} # id to next id in sequence
    # self.get_angles = {} # id to pose of the arm in said state
    self.get_pose = {} # id to end effector pose

    self.get_gripper_state = {} # id to 1/0 gripper open
    self.get_plan = {} # id to the motion plan that ends in getting to id pose
    self.first_movable_point_id = {} # pointer to the first moveable point in the sequence of waypoints
    self.names = [] # names of the joints of the rob

    self.ik_fail_pub = rospy.Publisher('ros_reality_ik_status', String, queue_size=0)
    self.gripper_pub = rospy.Publisher("/ein/right/forth_commands", String, queue_size = 0)
    self.gripper_pub.publish('openGripper')
    self.gripping = False
    # self.plan_pub = rospy.Publisher("ros_reality_motion_plan", MoveitPlan, queue_size = 0)
    self.generate_hardcode_locs()

  def generate_hardcode_locs(self):
    self.object_positions = []
    y_start = -0.561
    for i in xrange(3):
      seed = Point()
      seed.x = 0.686
      seed.y = y_start + float(i) * 0.1
      seed.z = -0.13
      self.object_positions.append(seed)

    print self.object_positions

  def send_plan_request(self, data):
    # print self.get_pose
    # print dataget_plan
    sid = data.sid.data
    gid = data.gid.data

    limb_joints = self.ik_solve_right(data.right_arm)
    
    # check that the pose is allowable
    if limb_joints == 0:
      print "fail"
      self.ik_fail_pub.publish(gid + " " + sid + " FAIL")
      return
    else:
      self.ik_fail_pub.publish(gid + " " + sid + " SUCCESS")

    # case where we are adding a point
    if gid not in self.get_pose:
      print gid
      self.network_group(data)

    if sid not in self.get_pose[gid]:
      print sid
      self.network_point(data)
    else:
      self.get_gripper_state[gid][sid] = data.right_open.data
      self.get_pose[gid][sid] = data.right_arm
    print self.get_gripper_state
    if data.visualize.data == '1':
      waypoints = [] # should use a global priority queue for this ideally...
      #waypoints.append(self.group.get_current_pose().pose)
      if gid not in self.first_movable_point_id:
        return

      # create the motion plan
      curr_id = self.first_movable_point_id[gid]
      
      while curr_id != None:
        waypoints.append(copy.deepcopy(self.get_pose[gid][curr_id]))
        curr_id = self.get_next_id[gid][curr_id]
      self.group = moveit_commander.MoveGroupCommander('right_arm')
      self.group.set_planning_time(10.0)
      (plan, fraction) = self.group.compute_cartesian_path(waypoints, 0.01, 0.0)

    '''
    #planning part of the functionPATH_TOLERANCE_VIOLATED
    if self.get_prev_id[data.gid.data][data.sid.data] == "START":
      self.group.set_start_state_to_current_state()
    else:
      self.set_start_pose(self.get_prev_id[data.gid.data][data.id.data])

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

    if self.get_next_id[data.gid.data][data.sid.data] != None: # case where there is a point after to move
      self.set_start_pose(data.id.data)
      self.group.set_planning_time(2.0)
      (plan, fraction) = self.group.compute_cartesian_path([self.get_pose[self.get_next_id[data.gid.data][data.sid.data]]], 0.01, 0.0)
      self.publish_plan(plan, data.id)
      self.get_plan[self.get_next_id[data.gid.data][data.sid.data]] = plan
    '''

  def move_to_goal(self, data):
    gids = data.data.split()
    print gids
    for gid in gids:
      curr = self.first_movable_point_id[gid]
      while curr != None:
        self.group.set_start_state_to_current_state()
        self.group.set_planning_time(2.0)

        if self.get_gripper_state[gid][curr] == "0" and not self.gripping: # case where we have to pick something up
          self.gripping = True
          (plan, fraction) = self.group.compute_cartesian_path(self.get_pick_points(self.get_pose[gid][curr]), 0.01, 0.0)
        else:
          (plan, fraction) = self.group.compute_cartesian_path([self.get_pose[gid][curr]], 0.01, 0.0)

        self.group.execute(plan)
        if self.get_gripper_state[gid][curr] == "1":
          self.gripping = False
          self.gripper_pub.publish('openGripper')
        else:
          self.gripper_pub.publish('closeGripper')
        rospy.sleep(0.8)
        curr = self.get_next_id[gid][curr]
      rospy.sleep(0.5)
    
  def get_pick_points(self, raw_pose):
    mini = float("inf")
    loc = None
    for pt in self.object_positions:
      d = self.get_dist(raw_pose.position, pt)
      if d < mini:
        mini = d
        loc = pt
    inter_pt = copy.copy(loc)
    inter_pt.z += 0.1
    inter_pose = Pose()
    loc_pose = Pose()
    inter_pose.position = inter_pt
    loc_pose.position = loc
    inter_pose.orientation = raw_pose.orientation
    loc_pose.orientation = raw_pose.orientation
    print [inter_pose, loc_pose]
    return [inter_pose, loc_pose]

  def get_dist(self, a, b):
    return math.sqrt( math.pow(a.x-b.x, 2) + math.pow(a.y-b.y, 2) + math.pow(a.z-b.z, 2) )

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
    sid = data.sid.data
    gid = data.gid.data 

    self.get_gripper_state[gid][sid] = data.right_open.data
    self.get_pose[gid][sid] = data.right_arm
    if data.prev_id.data != "":
      if data.prev_id.data == "START":
        self.get_prev_id[gid][sid] = "START"
        self.first_movable_point_id[gid] = sid
      else:
        self.get_next_id[gid][data.prev_id.data] = sid
        self.get_prev_id[gid][sid] = data.prev_id.data

    if data.next_id.data != "":
      self.get_prev_id[gid][data.next_id.data] = sid
      self.get_next_id[gid][sid] = data.next_id.data
    else:
      self.get_next_id[gid][sid] = None


  def network_group(self, data):
    gid = data.gid.data
    self.get_prev_id[gid] = {} 
    self.get_next_id[gid] = {} 
    self.get_pose[gid] = {} 
    self.get_gripper_state[gid] = {} 
    self.get_plan[gid] = {} 
    self.first_movable_point_id[gid] = {} 

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