#!/usr/bin/env python
import sys
import rospy
import moveit_commander
import geometry_msgs.msg
from moveit_msgs.msg import RobotTrajectory
from ros_reality_bridge.msg import MoveitTarget
from std_msgs.msg import String
import actionlib

from control_msgs.msg import GripperCommandAction, GripperCommandGoal


class GripperActionPlanner(object):
    def __init__(self, prefix="right"):
        self._prefix = prefix
        self._client = actionlib.SimpleActionClient(
            '/movo/%s_gripper_controller/gripper_cmd' % self._prefix,
            GripperCommandAction,
        )
        self._goal = GripperCommandGoal()
        server_up = self._client.wait_for_server(timeout=rospy.Duration(10))
        if not server_up:
            rospy.logerr("Timed out waiting for Gripper Command"
                         " Action Server to connect. Start the action server"
                         " before running example.")
            rospy.signal_shutdown("Timed out waiting for Action Server")
            sys.exit(1)
        self.clear()

    def command(self, position, block=False, timeout=15):
        self._goal.command.position = position
        self._goal.command.max_effort = -1.0
        self._client.send_goal(self._goal)
        if block:
            self._client.wait_for_result(timeout=rospy.Duration(timeout))

    def stop(self):
        self._client.cancel_goal()

    def wait(self, timeout=15):
        self._client.wait_for_result(timeout=rospy.Duration(timeout))

    def result(self):
        return self._client.get_result()

    def clear(self):
        self._goal = GripperCommandGoal()


class ArmPlanner(object):
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('movo_moveit_node', anonymous=True)
        self.robot = moveit_commander.RobotCommander()
        self.group_arms = moveit_commander.MoveGroupCommander('upper_body')
        self.left_ee_link = 'left_ee_link'
        self.right_ee_link = 'right_ee_link'
        self.group_arms.set_pose_reference_frame('/base_link')
        print 'planning frame:', self.group_arms.get_planning_frame()
        self.print_initializer_msgs()
        self.plan_publisher = rospy.Publisher('/movo_moveit/motion_plan', RobotTrajectory, queue_size=1)
        rospy.Subscriber('/ros_reality/goal_pose', MoveitTarget, self.goal_pose_callback, queue_size=1)
        rospy.Subscriber('/ros_reality/move_to_goal', String, self.execute_goal_callback, queue_size=1)
        self.plan_to_execute = None
        self.planning = False
        self.generate_identity_plan()

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
        assert isinstance(goal_pose_left, geometry_msgs.msg.PoseStamped)
        assert isinstance(goal_pose_right, geometry_msgs.msg.PoseStamped)
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
        self.generate_identity_plan(execute=False)
        return success


def identity_pose_request_callback(data):
    armPlanner.generate_identity_plan(execute=False)


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


if __name__ == '__main__':
    rospy.Subscriber('/holocontrol/identity_pose_request', String, identity_pose_request_callback)
    rospy.Subscriber('/holocontrol/right_gripper_command', String, right_gripper_callback)
    rospy.Subscriber('/holocontrol/left_gripper_command', String, left_gripper_callback)
    armPlanner = ArmPlanner()
    rightGripper = GripperActionPlanner("right")
    leftGripper = GripperActionPlanner("left")
    rospy.spin()
