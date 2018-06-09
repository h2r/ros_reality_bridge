#!/usr/bin/env python

import rospy
import moveit_commander
from std_msgs.msg import String
import moveit_msgs.msg
import geometry_msgs.msg
from ros_reality_bridge.msg import MoveitTarget
import sys
import copy
from moveit_msgs.msg import RobotState
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_python_interface', anonymous=True)
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group = moveit_commander.MoveGroupCommander('right_arm')
plans = []
#group = moveit_commander.MoveGroupCommander('both_arms') # 'both_arms'


def send_plan_request(data):
  print data
  #group.set_pose_target(data)
  

  # def set_start_state(self, msg):
  #        """
  #        Specify a start state for the group.
  #        Parameters
  #        ----------
  #        msg : moveit_msgs/RobotState
  #        Examples
  #        --------
  #        >>> from moveit_msgs.msg import RobotState
  #        >>> from sensor_msgs.msg import JointState
  #        >>> joint_state = JointState()
  #        >>> joint_state.header = Header()
  #        >>> joint_state.header.stamp = rospy.Time.now()
  #        >>> joint_state.name = ['joint_a', 'joint_b']
  #        >>> joint_state.position = [0.17, 0.34]
  #        >>> moveit_robot_state = RobotState()
  #        >>> moveit_robot_state.joint_state = joint_state
  #        >>> group.set_start_state(moveit_robot_state)
  #        """

# right_arm: 
#   position: 
#     x: 0.77
#     y: -0.311
#     z: 0.2146
#   orientation: 
#     x: 1.0
#     y: 0.0
#     z: 0.0
#     w: 0.0




  waypoints = []
  #waypoints.append(group.get_current_pose().pose)
  waypoints.append(copy.deepcopy(data.right_arm))
  end = geometry_msgs.msg.Pose()
  end.orientation.x = 1.0
  end.position.x = 0.74
  end.position.y = -0.158
  end.position.z = 0.3636
  waypoints.append(copy.deepcopy(end))
  print waypoints
  (plan, fraction) = group.compute_cartesian_path(waypoints, 0.01, 0.0)

  # joint_state = JointState()
  # joint_state.header = Header()
  # joint_state.header.stamp = rospy.Time.now()
  # joint_state.name = ['right_s0', 'right_s1', 'right_e0', 'right_e1', 'right_w0', 'right_w1', 'right_w2']
  # joint_state.position = [1.0748969631559835, -0.9011896944735679, -0.43517805553141176, 1.0331483510678539, 0.26593922628719313, 1.4965526530896336, 3.059]
  # moveit_robot_state = RobotState()
  # moveit_robot_state.joint_state = joint_state
  # group = moveit_commander.MoveGroupCommander('right_arm')
  # group.set_start_state(moveit_robot_state)
  # group.set_pose_target(data.right_arm, 'right_gripper')
  # group.set_planning_time(1.0)
  # plans.append(group)
  # for g in plans:
  #   g.plan()
  #   rospy.sleep(5.0)
  #plan = group.plan()
  #print plan

def move_to_goal(data):
  group.go(wait=True)


def main():
  #rospy.Subscriber('/ros_reality/goal_pose', geometry_msgs.msg.Pose, send_plan_request)
  rospy.Subscriber('/goal_pose', MoveitTarget, send_plan_request)
  rospy.Subscriber('/ros_reality/move_to_goal', String, move_to_goal)
  rospy.spin()



if __name__ == '__main__':
  try:
    main()
  except rospy.ROSInterruptException:
    moveit_commander.roscpp_shutdown()