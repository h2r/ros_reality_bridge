#!/usr/bin/env python
import numpy as np
import actionlib
import rospy
import tf
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import String
import nav_msgs.srv
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped


class Pose:
    def __init__(self, x=0.0, y=0.0, theta=0.0):
        self.x = x
        self.y = y
        self.theta = theta


class MovoTeleop:
    def __init__(self):
        self.rate = rospy.Rate(10)
        self.pose = Pose()
        # self.curr_state = 'standby'
        self.listener = tf.TransformListener()
        # self.ready_to_go = False
        self.pose_stamped = None

    def update_pose(self):
        while not rospy.is_shutdown():
            try:
                (trans, rot) = self.listener.lookupTransform('/map', '/base_link', rospy.Time(0))
                z_rot = tf.transformations.euler_from_quaternion(rot)[2]
                self.pose.x, self.pose.y = round(trans[0], 4), round(trans[1], 4)
                self.pose.theta = np.rad2deg(z_rot)
                self.pose_stamped = PoseStamped()
                self.pose_stamped.header.stamp = rospy.Time.now()
                self.pose_stamped.header.frame_id = '/map'
                self.pose_stamped.pose.position.x = trans[0]
                self.pose_stamped.pose.position.y = trans[1]
                self.pose_stamped.pose.position.z = trans[2]
                self.pose_stamped.pose.orientation.x = rot[0]
                self.pose_stamped.pose.orientation.y = rot[1]
                self.pose_stamped.pose.orientation.z = rot[2]
                self.pose_stamped.pose.orientation.w = rot[3]
                self.rate.sleep()
                return
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue

    # def move2goal(self, goal_x, goal_y, goal_theta):
    #     # assert self.curr_state == 'navigating'
    #     print 'moving to ({},{},{})'.format(goal_x, goal_y, goal_theta)
    #     try:
    #         result = movebase_client(goal_x, goal_y, goal_theta)
    #         if result:
    #             rospy.loginfo('Goal execution done!')
    #     except rospy.ROSInterruptException:
    #         rospy.loginfo('Navigation got interrupted.')


def move2goal(pose_stamped):
    try:
        result = movebase_client(pose_stamped)
        if result:
            rospy.loginfo('Goal execution done!')
    except rospy.ROSInterruptException:
        rospy.loginfo('Navigation got interrupted.')


def movebase_client(pose_stamped):
    move_base_client.wait_for_server()
    goal = MoveBaseGoal(target_pose=pose_stamped)
    move_base_client.send_goal(goal)
    wait = move_base_client.wait_for_result()
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
        movo_nav_finished_publisher.publish("")
        return move_base_client.get_result()


def waypoint_callback(data):
    waypoint_path = data
    # path_to_visualize = generate_navigation_plan(movo.pose_stamped, waypointPath.poses[0])
    for pose_stamped in waypoint_path.poses:
        pose_stamped.header.stamp = rospy.Time.now()  # TODO: is this necessary?
        path_to_visualize = generate_navigation_plan(movo.pose_stamped, pose_stamped)
        # print 'path:', path_to_visualize
        # movo_plan_publisher.publish(path_to_visualize)
        # print 'Simulated path published! Num poses:', len(path_to_visualize.poses)
        move2goal(pose_stamped)


def generate_navigation_plan(start, goal, tolerance=0.3):
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
        move_base_client = actionlib.SimpleActionClient('movo_move_base', MoveBaseAction)
        movo_pose_publisher = rospy.Publisher('holocontrol/ros_movo_pose_pub', String, queue_size=0)
        movo_plan_publisher = rospy.Publisher('holocontrol/simulated_nav_path', Path, queue_size=1)
        movo_nav_finished_publisher = rospy.Publisher('holocontrol/nav_finished', String, queue_size=1)
        rospy.Subscriber('holocontrol/unity_waypoint_pub', Path, waypoint_callback)
        path_planning_service = '/move_base/MovoGlobalPlanner/make_plan'
        rospy.loginfo('Waiting for GetPlan service...')
        rospy.wait_for_service(path_planning_service)
        rospy.loginfo('Got GetPlan service!')
        get_plan = rospy.ServiceProxy(path_planning_service, nav_msgs.srv.GetPlan)
        while not rospy.is_shutdown():
            movo.update_pose()
            movo_pose_publisher.publish('{},{},{}'.format(movo.pose.x, movo.pose.y, movo.pose.theta))
            movo.rate.sleep()
    except (rospy.ROSInterruptException, KeyboardInterrupt):
        print 'ROS exception :('
