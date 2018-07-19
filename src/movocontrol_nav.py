#!/usr/bin/env python
import numpy as np
import actionlib
import rospy
import tf
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import String
import nav_msgs.srv
from nav_msgs.msg import Path


class Pose:
    def __init__(self, x=0.0, y=0.0, theta=0.0):
        self.x = x
        self.y = y
        self.theta = theta


class MovoTeleop:
    def __init__(self):
        self.rate = rospy.Rate(10)
        self.pose = Pose()
        self.curr_state = 'standby'
        self.listener = tf.TransformListener()
        self.ready_to_go = False

    def update_pose(self):
        while not rospy.is_shutdown():
            try:
                (trans, rot) = self.listener.lookupTransform('/map', '/base_link', rospy.Time(0))
                z_rot = tf.transformations.euler_from_quaternion(rot)[2]
                self.pose.x, self.pose.y = round(trans[0], 4), round(trans[1], 4)
                self.pose.theta = np.rad2deg(z_rot)
                self.rate.sleep()
                return
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue

    def move2goal(self, goal_x, goal_y, goal_theta):
        assert self.curr_state == 'navigating'
        print 'moving to ({},{},{})'.format(goal_x, goal_y, goal_theta)
        try:
            result = movebase_client(goal_x, goal_y, goal_theta)
            if result:
                rospy.loginfo('Goal execution done!')
        except rospy.ROSInterruptException:
            rospy.loginfo('Navigation test finished.')


def movebase_client(goal_x, goal_y, goal_theta):
    client = actionlib.SimpleActionClient('movo_move_base', MoveBaseAction)
    client.wait_for_server()

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = goal_x
    goal.target_pose.pose.position.y = goal_y

    print 'GOAL:', goal_theta
    yaw = np.deg2rad(goal_theta)
    quaternion = tf.transformations.quaternion_from_euler(0, 0, yaw)
    print 'Q GOAL:', quaternion
    goal.target_pose.pose.orientation.x = quaternion[0]
    goal.target_pose.pose.orientation.y = quaternion[1]
    goal.target_pose.pose.orientation.z = quaternion[2]
    goal.target_pose.pose.orientation.w = quaternion[3]

    client.send_goal(goal)
    wait = client.wait_for_result()
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
        return client.get_result()


def waypoint_callback(data):
    print 'waypoint callback:', data.data
    pose = data.data.split(';')
    print 'pose:', pose
    movo.curr_state = 'navigating'
    # print 'set state to navigating'
    for p in pose:
        pose_data = p.split(',')
        assert len(pose_data) == 3
        goal_x = float(pose_data[0])
        goal_y = float(pose_data[1])
        goal_theta = float(pose_data[2])
        movo.move2goal(goal_x, goal_y, goal_theta)
    # print 'set state to standby'
    movo.curr_state = 'standby'


def state_request_callback(data):
    if not movo.ready_to_go:
        print 'Movocontrol is initialized and ready to go!'
        movo.ready_to_go = True
    assert (movo.curr_state in ['standby', 'navigating'])
    movo_state_publisher.publish(movo.curr_state)


def poserequest_callback(data):
    # print 'poserequest_callback'
    movo.update_pose()
    movo_pose_publisher.publish('{},{},{}'.format(movo.pose.x, movo.pose.y, movo.pose.theta))


def initialize_request_callback(data):
    print 'resetting movocontrol_nav...'
    movo.ready_to_go = False


def generate_navigation_plan(start, goal, tolerance):
    """
    :type start: geometry_msgs.msg.PoseStamped
    :type goal: geometry_msgs.msg.PoseStamped
    :type tolerance: float
    :return: nav_msgs.msg.Path
    """
    try:
        return get_plan(start, goal, tolerance)
    except rospy.ServiceException as exc:
        print 'Service did not process request: ' + str(exc)


if __name__ == '__main__':
    try:
        listener = tf.TransformListener()
        rospy.init_node('movo_controller', anonymous=True)
        movo = MovoTeleop()
        movo_state_publisher = rospy.Publisher('holocontrol/ros_movo_state_pub', String, queue_size=0)
        movo_pose_publisher = rospy.Publisher('holocontrol/ros_movo_pose_pub', String, queue_size=0)
        movo_plan_publisher = rospy.Publisher('move_base/GlobalPlanner/plan', Path, queue_size=0)
        rospy.Subscriber('holocontrol/movo_state_request', String, state_request_callback)
        rospy.Subscriber('holocontrol/movo_pose_request', String, poserequest_callback)
        rospy.Subscriber('holocontrol/unity_waypoint_pub', String, waypoint_callback)
        rospy.Subscriber('holocontrol/init_movocontrol_request', String, initialize_request_callback)
        path_planning_service = '/move_base/GlobalPlanner/make_plan'
        get_plan = rospy.ServiceProxy(path_planning_service, nav_msgs.srv.GetPlan)
    except (rospy.ROSInterruptException, KeyboardInterrupt):
        print 'ROS exception :('
    rospy.spin()
