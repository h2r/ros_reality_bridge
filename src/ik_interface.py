#!/usr/bin/env python

import rospy
import baxter_interface

from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion

from std_msgs.msg import Header, String

from baxter_core_msgs.srv import SolvePositionIK, SolvePositionIKRequest


def ik_solve(limb, point, quaternion):
    ns = "ExternalTools/" + limb + "/PositionKinematicsNode/IKService"
    iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
    ikreq = SolvePositionIKRequest()

    hdr = Header(stamp=rospy.Time.now(), frame_id='base')
    poses = {
        str(limb): PoseStamped(header=hdr,
            pose=Pose(position=point, orientation=quaternion))}
    ikreq.pose_stamp.append(poses[limb])
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

def ik_handler(limb, msg):
    # remove the moveToEEPose at the end, and convert each string to float
    msg_split = msg.strip().split(' ')
    if len(msg_split) < 3:
        return
    x, y, z, qx, qy, qz, qw = [float(elem) for elem in msg_split[0:7]]
    limb_joints = ik_solve(limb, Point(x, y, z), Quaternion(qx, qy, qz, qw))
    if limb_joints:
        if limb == 'right':
            right_limb.set_joint_positions(limb_joints)
        else:
            left_limb.set_joint_positions(limb_joints)
    else:
        ik_fail_pub.publish('f')

def gripper_handler(limb, msg):
    if 'openGripper' in msg:
        if limb == 'right':
            print "opening right gripper"
            right_gripper.open()
        if limb == 'left':
            print "opening left gripper"
            left_gripper.open()
    if 'closeGripper' in msg:
        if limb == 'right':
            right_gripper.close()
        if limb == 'left':
            left_gripper.close()

def pose_request_callback(data):
    # determine limb
    if 'right' in data._connection_header['topic']:
        limb = 'right'
    else:
        limb = 'left'

    # get message
    msg = data.data
    print 'msg:', msg

    # call handle ik
    ik_handler(limb, msg)

    # call gripper handler
    gripper_handler(limb, msg)
    

def main():
    global right_limb, right_gripper
    global left_limb, left_gripper, ik_fail_pub
    
    ik_fail_pub = rospy.Publisher('ros_reality_ik_status', String, queue_size=0)

    rospy.init_node('ros_reality_ik_interface')

    right_limb = baxter_interface.Limb('right')
    right_gripper = baxter_interface.Gripper('right')

    left_limb = baxter_interface.Limb('left')
    left_gripper = baxter_interface.Gripper('left')

    # gripper calibrating 
    if not right_gripper.calibrated():
        right_gripper.calibrate()
    if not left_gripper.calibrated():
        left_gripper.calibrate()

    right_gripper.open()
    left_gripper.open()

    rospy.Subscriber('/ein/right/forth_commands', String, pose_request_callback)
    rospy.Subscriber('/ein/left/forth_commands', String, pose_request_callback)

    rospy.spin()


if __name__ == '__main__':
    main()
