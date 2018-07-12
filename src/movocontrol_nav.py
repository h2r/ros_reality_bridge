import roslib
import rospy
import math
import tf
import geometry_msgs.msg
import numpy as np
from math import pow, atan2, sqrt
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from std_msgs.msg import String
import sys
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal


class Pose:
    def __init__(self, x=0.0, y=0.0, theta=0.0):
        self.x = x
        self.y = y
        self.theta = theta

class MovoTeleop:
    def __init__(self):
        self.rate = rospy.Rate(10.0)
        self.pose = Pose()
        self.curr_state = 'standby'
        listener = tf.TransformListener()

    def update_pose(self):
        while not rospy.is_shutdown():
            try:
                (trans, rot) = listener.lookupTransform('/map', '/base_link', rospy.Time(0))
                #print 'Trans:', trans
                z_rot = tf.transformations.euler_from_quaternion(rot)[2]
                self.pose.x, self.pose.y = round(trans[0], 4), round(trans[1], 4)
                self.pose.theta = np.rad2deg(z_rot)
                self.rate.sleep()
                #return (self.pose.x, self.pose.y, self.pose.theta)
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

    # TODO: input goal_theta as target orientation
    # -------------------------------------------------------------
    #goal.target_pose.pose.orientation.w = 1.0
    print 'GOAL:', goal_theta
    yaw = np.deg2rad(goal_theta)
    quaternion = tf.transformations.quaternion_from_euler(0, 0, yaw)
    print 'Q GOAL:', quaternion
    goal.target_pose.pose.orientation.x = quaternion[0]
    goal.target_pose.pose.orientation.y = quaternion[1]
    goal.target_pose.pose.orientation.z = quaternion[2]
    goal.target_pose.pose.orientation.w = quaternion[3]
    #goal.target_pose.pose.orientation.y = np.deg2rad(goal_theta) # TODO: check if this works.
    # -------------------------------------------------------------

    client.send_goal(goal)
    wait = client.wait_for_result()
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
        return client.get_result()

def unit(v):
    v = np.asarray(v)
    return v / np.linalg.norm(v)

def del_angle(v1, v2):
    v1 = np.asarray(v1)
    v2 = np.asarray(v2)
    u1 = v1 / np.linalg.norm(v1)
    u2 = v2 / np.linalg.norm(v2)
    s = np.linalg.norm(np.cross(v1, v2))
    c = np.dot(v1, v2)
    return np.arctan2(s, c)

def waypoint_callback(data):
    print 'waypoint callback:', data.data
    pose = data.data.split(';')
    print 'pose:', pose
    movo.curr_state = 'navigating'
    print 'set state to navigating'
    for p in pose:
        pose_data = p.split(',')
        assert len(pose_data) == 3
        goal_x = float(pose_data[0])
        goal_y = float(pose_data[1])
        goal_theta = float(pose_data[2])
        movo.move2goal(goal_x, goal_y, goal_theta)
    #coords = data.data.split(';')
    #print 'coords:', coords
    #movo.curr_state = 'navigating'
    #print 'set state to navigating'
    #for coord in coords:
    #    coord = coord.split(',')
    #    assert len(coord) == 2
    #    goal_x = float(coord[0])
    #    goal_y = float(coord[1])
    #    movo.move2goal(goal_x, goal_y)
    print 'set state to standby'
    movo.curr_state = 'standby'

def state_request_callback(data):
    print 'state_request_callback:', movo.curr_state
    assert (movo.curr_state in ['standby', 'navigating'])
    movo_state_publisher.publish(movo.curr_state)

def poserequest_callback(data):
    print 'poserequest_callback'
    movo.update_pose()
    movo_pose_publisher.publish('{},{},{}'.format(movo.pose.x, movo.pose.y, movo.pose.theta))

if __name__ == '__main__':
    try:
        listener = tf.TransformListener()
        #max_speed = 0.2
        #min_speed = 0.1
        #rotation_tolerance = 0.05
        #distance_tolerance = 0.15
        rospy.init_node('movo_controller', anonymous=True)
        velocity_publisher = rospy.Publisher('/movo/cmd_vel', Twist, queue_size=10)
        movo_state_publisher = rospy.Publisher('holocontrol/ros_movo_state_pub', String, queue_size=0)
        movo_pose_publisher = rospy.Publisher('holocontrol/ros_movo_pose_pub', String, queue_size=0)
        rospy.Subscriber('holocontrol/movo_state_request', String, state_request_callback)
        rospy.Subscriber('holocontrol/movo_pose_request', String, poserequest_callback)
        rospy.Subscriber('holocontrol/unity_waypoint_pub', String, waypoint_callback)
        movo = MovoTeleop()
        movo.curr_state = 'standby'
    except (rospy.ROSInterruptException, KeyboardInterrupt):
        print 'ROS exception :('
    rospy.spin()
