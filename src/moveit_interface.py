import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

def sendPlanRequest(data):
  group.set_pose_target(data.left_arm, 'left_arm')
  group.set_pose_target(data.right_arm, 'right_arm')
  plan1 = group.plan()

def main():
  moveit_commander.roscpp_initialize(sys.argv)
  rospy.init_node('move_group_python_interface', anonymous=True)
  robot = moveit_commander.RobotCommander()
  scene = moveit_commander.PlanningSceneInterface()
  group = moveit_commander.MoveGroupCommander("both_arms")


if __name__ == '__main__':
  main()