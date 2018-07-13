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

'''
This class is responsible for handling incoming requests for the Unity front end, it handles
plan requests (for visualization of a motion trajectory) and move requests (which actuates
the baxter as per the moveit plan)
'''
class PlanHandler(object):

  def __init__(self):
    self.initializer()


  '''
  a helpert that is essentially the constructor. It is used to init the moveit commander, set 
  up publisehrs, and construct that dicts that are necessary for bookeeping all of the waypoints
  '''
  def initializer(self):
    # moveit commander init
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_python_interface', anonymous=True)
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    self.group = moveit_commander.MoveGroupCommander('right_arm')

    # all following dicts are indexed by group ID (gid) and then by gripper ID (sid) to get
    # the desired information
    self.get_prev_id = {} # given gid and sid, stores the sid of the previous gripper in the sequence
    self.get_next_id = {} # stores the sid of the next gripper in the sequence
    # self.get_angles = {} 
    self.get_pose = {} # stores the pose (position and quaternion) of the gripper
    self.get_gripper_state = {} # stores the gipper state (1 if open else 0)
    self.get_plan = {} # not used in current version of the code
    self.first_movable_point_id = {} # stores pointers to the first waypoint in each group

    # names of the joints of the robot (not currently used)
    self.names = [] 

    self.ik_fail_pub = rospy.Publisher('ros_reality_ik_status', String, queue_size=0)
    self.gripper_pub = rospy.Publisher("/ein/right/forth_commands", String, queue_size = 0)
    self.gripper_pub.publish('openGripper')

    # bool to keep track of if the right gripper is gripping (holding) and object 
    self.gripping = False

    # self.plan_pub = rospy.Publisher("ros_reality_motion_plan", MoveitPlan, queue_size = 0)

    # helper to give the robot the locations of the objects in the scene
    self.generate_hardcode_locs()


  '''
  Helper method to give the locations of the objects on the table. This was done by getting the xyz
  EE pose from ein
  '''
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


  '''
  Callback function used when a plan request is made from the front end
  '''
  def send_plan_request(self, data):
    # saving the gid and the sid associated with the gripper
    sid = data.sid.data
    gid = data.gid.data
    
    # check that the pose is allowable, IK failures
    limb_joints = self.ik_solve_right(data.right_arm)
    if limb_joints == 0:
      print "fail"
      self.ik_fail_pub.publish(gid + " " + sid + " FAIL")
      return
    else:
      self.ik_fail_pub.publish(gid + " " + sid + " SUCCESS")

    # case where we are handling a gid we have not seen before
    if gid not in self.get_pose:
      print gid
      self.network_group(data)
    # case where we have a sid we have not seen before
    if sid not in self.get_pose[gid]:
      print sid
      self.network_point(data)
    else: # case where we have a sid that we have seen before
      self.get_gripper_state[gid][sid] = data.right_open.data
      self.get_pose[gid][sid] = data.right_arm

    # visualize is a feild that is true if we want to generate moveit plans to be
    # displayed in the frontend. This is true ("1") except when we are making a copy
    # of a group with the same gripper locations but with different a different gid and
    # sids. In this case we do not want to viusalize the plan, but do want to network in
    # all of the points.
    if data.visualize.data == '1':
      waypoints = []
      if gid not in self.first_movable_point_id:
        return

      # loop through to get all of the waypoints associated with a group
      curr_id = self.first_movable_point_id[gid]      
      while curr_id != None:
        waypoints.append(copy.deepcopy(self.get_pose[gid][curr_id]))
        curr_id = self.get_next_id[gid][curr_id]

      # make the moveit plan request to the right arm
      self.group = moveit_commander.MoveGroupCommander('right_arm')
      self.group.set_planning_time(10.0)
      (plan, fraction) = self.group.compute_cartesian_path(waypoints, 0.01, 0.0)

    # Old code for when each part (from one waypoint to the next) was planned seperately
    # this might be useful in the future but does not incorporate the notion of gid and
    # hense will have to be tweeked if used. 
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

  '''
  Callback function used when there is a request to actuate the joints. We replan between each pair of waypoints in order to 
  open and close the grippers at the waypoints.  
  '''
  def move_to_goal(self, data):
    # the data parameter has a string of all the groups that need to be executed
    gids = data.data.split()
    print gids
    for gid in gids:
      curr = self.first_movable_point_id[gid]
      while curr != None:
        # replan based on waypoints associated with the group
        self.group.set_start_state_to_current_state()
        self.group.set_planning_time(2.0)

        if self.get_gripper_state[gid][curr] == "0" and not self.gripping: # case where we have to pick something up
          self.gripping = True
          # call helper to find the closest object to the user specified waypoint
          (plan, fraction) = self.group.compute_cartesian_path(self.get_pick_points(self.get_pose[gid][curr]), 0.01, 0.0)
        else: # case where we do not need to pick something up
          (plan, fraction) = self.group.compute_cartesian_path([self.get_pose[gid][curr]], 0.01, 0.0)
        # execute segment of the plan
        self.group.execute(plan)

        # open or close gripper as necessary
        if self.get_gripper_state[gid][curr] == "1":
          self.gripping = False
          self.gripper_pub.publish('openGripper')
        else:
          self.gripper_pub.publish('closeGripper')
        rospy.sleep(0.8)
        curr = self.get_next_id[gid][curr]
      rospy.sleep(0.5)
    
  '''
  Helper to find the closest object location to raw_pose deemed to be a place where an object is to be picked up
  '''
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
    return [inter_pose, loc_pose] # returns an intermediate point above the object and the location of the object

  # helper to find the distance between 2 points
  def get_dist(self, a, b):
    return math.sqrt( math.pow(a.x-b.x, 2) + math.pow(a.y-b.y, 2) + math.pow(a.z-b.z, 2) )

  # Old function to set the start state of joints, no longer used, but could be useful. This would aslo need to be
  # adapted to fit the notion of groups
  '''
  def set_start_pose(self, id_data):
    joint_state = JointState()
    joint_state.header = Header()
    joint_state.header.stamp = rospy.Time.now()
    assert len(self.names) != 0
    joint_state.name = self.names
    joint_state.position = self.get_angles[id_data]
    moveit_robot_state = RobotState()
    moveit_robot_state.joint_state = joint_state
    self.group.set_start_state(moveit_robot_state)
  '''

  '''
  Helper to add a new waypoint (sid) to the dicts
  '''
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


  '''
  Helper to add a new group (gid) to the dicts
  '''
  def network_group(self, data):
    gid = data.gid.data
    self.get_prev_id[gid] = {} 
    self.get_next_id[gid] = {} 
    self.get_pose[gid] = {} 
    self.get_gripper_state[gid] = {} 
    self.get_plan[gid] = {} 
    self.first_movable_point_id[gid] = {} 

  '''
  Helper adapted from the ik_interface.py to check for ik failures
  '''
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

  '''
  Function called in the main line to start the subs to plan requests and execute requests
  from the front end
  '''
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