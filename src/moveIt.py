import sys
import rospy
import moveit_commander
from moveit_msgs.msg import CollisionObject
import moveit_msgs.msg
import geometry_msgs.msg

print ("==== Initializing ===== ")
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_python_interface_tutorial',anonymous=True)

robot = moveit_commander.RobotCommander()
#scene = moveit_commander.PlanningSceneInterface()
group = moveit_commander.MoveGroupCommander("left_arm")

display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',moveit_msgs.msg.DisplayTrajectory)


print ("==== Waiting for Rviz ==== ")
#rospy.sleep(10)
print ("==== Starting ===== ")

print ("==== Robot State:\n")
#print robot.get_current_state()


print ("==== PLAN 1 (STARTING POST -> L1 VIA END EFFECTOR POSITION) ====")
pose_target_l1 = geometry_msgs.msg.Pose()
pose_target_l1.orientation.x = -.459
pose_target_l1.orientation.y = .888
pose_target_l1.orientation.z = 0
pose_target_l1.orientation.w = 0
pose_target_l1.position.x = .479
pose_target_l1.position.y = .739
pose_target_l1.position.z = .433

group.set_pose_target(pose_target_l1)

plan1 = group.plan()
print plan1
#group.go(wait=True)

#rospy.sleep(1)

pose_target_l1 = geometry_msgs.msg.Pose()
pose_target_l1.orientation.x = -.22
pose_target_l1.orientation.y = .968
pose_target_l1.orientation.z = -.03
pose_target_l1.orientation.w = .109
pose_target_l1.position.x = .824
pose_target_l1.position.y = .533
pose_target_l1.position.z = .33

group.set_pose_target(pose_target_l1)

plan1 = group.plan()
#print plan1
#group.go(wait=True)
"""
print ("==== Compiling plan ====")
#rospy.sleep(5)

print ("==== Visualizing ==== ")

#display_trajectory = moveit_msgs.msg.DisplayTrajectory()

#display_trajectory.trajectory_start = robot.get_current_state()
#display_trajectory.trajectory.append(plan1)
#display_trajectory_publisher.publish(display_trajectory)

print ("===PLAN 2 (L1 -> L2 VIA JOINT ANGLE CONTROL)====")
group.clear_pose_targets()

group_variable_values_l1 = group.get_current_joint_values()
group_variable_values_l2 = group.get_current_joint_values()
print "JOINT VALUES: ", group_variable_values_l2

group_variable_values_l1[0] = -.36 
group_variable_values_l1[1] = -.90
group_variable_values_l1[2] = .74
group_variable_values_l1[3] = 1.37
group_variable_values_l1[4] = -.511
group_variable_values_l1[5] = 1.10
group_variable_values_l1[6] = .50


group_variable_values_l2[0] = -.93 
group_variable_values_l2[1] = -.42
group_variable_values_l2[2] = .68
group_variable_values_l2[3] = .81
group_variable_values_l2[4] = -.65
group_variable_values_l2[5] = 1.27
group_variable_values_l2[6] = .25

group.set_joint_value_target(group_variable_values_l2)

plan2 = group.plan()
print plan2.joint_trajectory.points

print ("==== Visualizing ==== ")

#display_trajectory = moveit_msgs.msg.DisplayTrajectory()

#display_trajectory.trajectory_start = robot.get_current_state()
#display_trajectory.trajectory.append(plan2)
#display_trajectory_publisher.publish(display_trajectory)

group.go(wait=True)

print("The clocks of time continue to tick...")
rospy.spin()
"""
