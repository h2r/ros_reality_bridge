#!/usr/bin/env python
import sys
import rospy
import moveit_commander
import geometry_msgs.msg
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import RobotTrajectory
from ros_reality_bridge.msg import MoveitTarget
from std_msgs.msg import String
import actionlib
from control_msgs.msg import GripperCommandAction, GripperCommandGoal
from ar_track_alvar_msgs.msg import AlvarMarkers
import tf
import numpy as np
import random


class ArmPlanner(object):
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        #rospy.init_node('movo_moveit_node', anonymous=True)
        self.robot = moveit_commander.RobotCommander()
        self.group_arms = moveit_commander.MoveGroupCommander('upper_body')
        print 'planning frame:', self.group_arms.get_planning_frame()
        self.group_arms.set_pose_reference_frame('/odom')
        self.left_ee_link = 'left_ee_link'
        self.right_ee_link = 'right_ee_link'
        self.print_initializer_msgs()
        self.plan_publisher = rospy.Publisher('/movo_moveit/motion_plan', RobotTrajectory, queue_size=1)
        rospy.Subscriber('/ros_reality/goal_pose', MoveitTarget, self.goal_pose_callback, queue_size=1)
        rospy.Subscriber('/ros_reality/move_to_goal', String, self.execute_goal_callback, queue_size=1)
        self.plan_to_execute = None
        self.planning = False

    def print_initializer_msgs(self):
        print "================ Robot Groups ==============="
        print self.robot.get_group_names()
        print "============================================="

    def goal_pose_callback(self, data):
        self.planning = True
        moveit_target = data
        print 'planning...'
        goal_pose_left = moveit_target.left_arm
        goal_pose_right = moveit_target.right_arm
        assert isinstance(goal_pose_left, geometry_msgs.msg.PoseStamped)
        assert isinstance(goal_pose_right, geometry_msgs.msg.PoseStamped)
        plan = self.generate_plan(goal_pose_left=goal_pose_left, goal_pose_right=goal_pose_right)
        self.plan_to_execute = plan
        self.planning = False
        print 'done planning!'

    def execute_goal_callback(self, data):
        while self.planning:
            continue
        if self.plan_to_execute is None:
            print 'execute_goal_callback: no plan to execute!'
            return
        self.execute_plan(self.plan_to_execute)

    def get_pose_right_arm(self):
        """
        Get the pose of the right end-effector.
        :return: geometry_msgs.msg.Pose
        """
        return self.group_arms.get_current_pose(self.right_ee_link)

    def get_pose_left_arm(self):
        """
        Get the pose of the left end-effector.
        :return: geometry_msgs.msg.Pose
        """
        return self.group_arms.get_current_pose(self.left_ee_link)

    def generate_plan(self, goal_pose_left=None, goal_pose_right=None):
        """
        Moves the group ('left_arm' or 'right_arm') to goal_pose
        :return: RobotTrajectory (None if failed)
        :type goal_pose_left: geometry_msgs.msg.PoseStamped
        :type goal_pose_right: geometry_msgs.msg.PoseStamped
        """
        if goal_pose_left is None and goal_pose_right is None:
            return
        if goal_pose_left is not None:
            self.group_arms.set_pose_target(goal_pose_left, self.left_ee_link)
        if goal_pose_right is not None:
            self.group_arms.set_pose_target(goal_pose_right, self.right_ee_link)
        print 'planning...'
        plan = self.group_arms.plan()
        if not plan.joint_trajectory.joint_names:  # empty list means failed plan
            print 'Plan failed! :('
            return
        self.plan_publisher.publish(plan)
        return plan

    def generate_identity_plan(self, execute=False):
        """
        Generate a plan to move to the current pose - used to force joint state updates in unity.
        :return: moveit_msgs.msg.RobotTrajectory (None if failed)
        """
        print 'generating identity plan...'
        pose_right = self.get_pose_right_arm()
        pose_left = self.get_pose_left_arm()
        print 'RIGHT POSE:', self.get_pose_right_arm()
        plan = self.generate_plan(goal_pose_left=pose_left, goal_pose_right=pose_right)
        if execute:
            print 'execute status:', self.execute_plan(plan)
        print 'done!'
        return plan

    def execute_plan(self, plan):
        """
        Execute the group's plan.
        :type plan: RobotTrajectory
        :return: A boolean indicating success of movement execution.
        """
        if plan is None:
            print 'No plan to execute!'
            return False
        assert isinstance(plan, RobotTrajectory)
        success = self.group_arms.execute(plan)
        return success


def right_gripper_callback(data):
    assert data.data in ['open', 'close']
    position = 0.165
    if data.data == 'close':
        position = 0.0
    rightGripper.command(position)
    rightGripper.wait()
    print 'Gripper command result:', rightGripper.result()


def left_gripper_callback(data):
    assert data.data in ['open', 'close']
    position = 0.165
    if data.data == 'close':
        position = 0.0
    leftGripper.command(position)
    leftGripper.wait()
    print 'Gripper command result:', leftGripper.result()

def ar_tag_callback(data):
    frame = data.header.frame_id
    print 'frame:', frame

def generate_pose(x, y, z, qx, qy, qz, qw):
    pose = PoseStamped()
    pose.header.stamp = rospy.Time.now()
    pose.header.frame_id = 'odom'
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = z
    pose.pose.orientation.x = qx
    pose.pose.orientation.y = qy
    pose.pose.orientation.z = qz
    pose.pose.orientation.w = qw
    return pose

def generate_pose(position, orientation):
    pose = PoseStamped()
    pose.header.stamp = rospy.Time.now()
    pose.header.frame_id = '/odom'
    pose.pose.position.x = position[0]
    pose.pose.position.y = position[1]
    pose.pose.position.z = position[2]
    pose.pose.orientation.x = orientation[0]
    pose.pose.orientation.y = orientation[1]
    pose.pose.orientation.z = orientation[2]
    pose.pose.orientation.w = orientation[3]
    return pose

def move_arm(plan):
    global armPlanner
    if not plan:
        return False
    return armPlanner.execute_plan(plan)

def dump_cup():
    global armPlanner
    group = armPlanner.group_arms
    joints = group.get_current_joint_values()
    joints[-1] += np.pi
    if joints[-1] > np.pi:
        joints[-1] -= 2*np.pi
    if joints[-1] < -np.pi:
        joints[-1] += 2*np.pi
    group.set_joint_value_target(joints)
    plan = group.plan()
    return armPlanner.execute_plan(plan)

def gohome():
    global upright_orientation, armPlanner
    joints = [0.0246072206646204, 1.5699611594770477, 1.3755988694331027, -0.40243985970179796, 2.6155155402810664, 0.024652774019944257, -0.4529352581824634, 1.736052130337038, 0.02641536109149456, -0.030099578201770782, -2.6349187993511656, 1.393446309794378, 2.9582854115468766, -1.6660002078655398, -3.0842920795756488, 0.01734649976023112, 0.7757197033638823]
    armPlanner.group_arms.set_joint_value_target(joints)
    plan = armPlanner.group_arms.plan()
    if not armPlanner.execute_plan(plan):
        print 'move home failed!'
    #position = [0.9, -0.3, 1.0]
    #pose = generate_pose(position, upright_orientation)
    #plan = armPlanner.generate_plan(None, pose)
    #if not armPlanner.execute_plan(plan):
    #    print 'move home failed!'

def pose1():
    global upright_orientation, armPlanner
    position = [0.9, -0.1, 1.0]
    pose = generate_pose(position, upright_orientation)
    plan = armPlanner.generate_plan(None, pose)
    if not armPlanner.execute_plan(plan):
        print 'pose1 failed!'

def pose2():
    global upright_orientation, armPlanner
    position = [0.9, 0.2, 1.0]
    pose = generate_pose(position, upright_orientation)
    plan = armPlanner.generate_plan(None, pose)
    if not armPlanner.execute_plan(plan):
        print 'pose2 failed!'

def generate_coords(num_objects):
    global all_coords
    coords = random.sample(all_coords, num_objects)
    return coords

def generate_all_coords():
    shelves = 4
    rows = 3
    cols = 15
    coords = []
    for s in xrange(shelves):
        for r in xrange(rows):
            for c in xrange(cols):
                coords.append('{},{},{}'.format(s, r, c))
    return coords


if __name__ == '__main__':
    global all_coords, armPlanner, upright_orientation, dump_orientation
    all_coords = generate_all_coords()
    upright_orientation = [-0.980485293277, -0.15731597732, -0.00049072175508, 0.117898397529]
    #dump_orientation = [-0.195536322278, -0.0479014920223, 0.160869170862, 0.966225700133]
    armPlanner = ArmPlanner()
    rospy.init_node('movo_shelf_experiment_node')
    coord_publisher = rospy.Publisher('holocontrol/shelf_coords', String, queue_size=0)
    print 'waiting for connection...'
    while coord_publisher.get_num_connections() == 0:
        rospy.sleep(1)
    print 'connection obtained!'

    NUM_TRIALS = 1
    NUM_TO_PICK = 10
    for trial_num, trial in enumerate(xrange(NUM_TRIALS)):
        coords = generate_coords(NUM_TO_PICK)
        print 'Coordinates generated!'
        status = raw_input('Press any key to begin trial {}/{}.'.format(trial_num+1, NUM_TRIALS))
        for i, c in enumerate(coords):
            print 'Picking object {}/{}'.format(i+1, len(coords))
            print 'coord: ({})'.format(c)
            coord_publisher.publish(c)
            print 'published', c
            # Wait for subject to pick the item.
            if (i < 9):
                status = raw_input('Press enter to continue, or type quit to exit: ')
                if status.lower() == 'quit':
                    sys.exit(1)
        print 'Trials complete!'

    #gohome()
    #pose1()
    #pose2()
    #dump_cup()
    #gohome()


    #while True:
    #    joints = armPlanner.group_arms.get_current_joint_values()
    #    print joints
