#!/usr/bin/env python
"""
Example for commanding robot without moveit
"""

import sys

import numpy as np
import rospy
from geometry_msgs.msg import Pose, Point, Quaternion
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from std_msgs.msg import Empty
from trac_ik_python.trac_ik import IK

GRIPPER_LINK = "gripper_link"
ARM_BASE_LINK = "arm_base_link"
MOVE_GROUP_NAME = 'arm'

ROSTOPIC_SET_ARM_JOINT = '/goal_dynamixel_position'
ROSTOPIC_SET_PAN_JOINT = '/pan/command'
ROSTOPIC_SET_TILT_JOINT = '/tilt/command'
ROSTOPIC_OPEN_GRIPPER = '/gripper/open'
ROSTOPIC_CLOSE_GRIPPER = '/gripper/close'

IK_POSITION_TOLERANCE = 0.01
IK_ORIENTATION_TOLERANCE = np.pi/9
current_joint_state = None


def home_arm(pub):
    rospy.loginfo('Going to arm home pose')
    set_arm_joint(pub, np.zeros(5))
    rospy.sleep(5)

def set_camera_pan(pub, pan_rad):
    pan_msg = Float64()
    pan_msg.data = pan_rad
    rospy.loginfo('Going to camera pan: {} rad'.format(pan_rad))
    pub.publish(pan_msg)

def set_camera_tilt(pub, tilt_rad):
    tilt_msg = Float64()
    tilt_msg.data = tilt_rad
    rospy.loginfo('Going to camera tilt: {} rad'.format(tilt_rad))
    pub.publish(tilt_msg)

def set_arm_joint(pub, joint_target):
    joint_state = JointState()
    joint_state.position = tuple(joint_target)
    pub.publish(joint_state)

def get_joint_state(data):
    global current_joint_state
    if len(data.position) == 11:
    	current_joint_state = data.position[4:9]

def open_gripper(pub):
    empty_msg = Empty()
    rospy.loginfo('Opening gripper')
    pub.publish(empty_msg)

def close_gripper(pub):
    empty_msg = Empty()
    rospy.loginfo('Closing gripper')
    pub.publish(empty_msg)


def compute_ik(ik_solver, target_pose, current_joint):
    """
    Parameters
    ----------
    ik_solver: trac_ik_python.trac_ik Ik object
    target_pose: type geometry_msgs/Pose
    current_joint: list with length the number of joints (i.e. 5)
    Returns
    ----------
    IK solution (a list of joint angles for target_pose)
    if found, None otherwise
    """
    result = ik_solver.get_ik(current_joint,
                              target_pose.position.x,
                              target_pose.position.y,
                              target_pose.position.z,
                              target_pose.orientation.x,
                              target_pose.orientation.y,
                              target_pose.orientation.z,
                              target_pose.orientation.w,
                              IK_POSITION_TOLERANCE,
                              IK_POSITION_TOLERANCE,
                              IK_POSITION_TOLERANCE,
                              IK_ORIENTATION_TOLERANCE,
                              IK_ORIENTATION_TOLERANCE,
                              IK_ORIENTATION_TOLERANCE)

    if result:
        rospy.loginfo('IK solution found')
    else:
        rospy.logerr('No IK solution found')
    return result

def main():
    rospy.init_node('IK_Control', anonymous=True)

    #target_poses = [Pose(Point(0.279, 0.176, 0.217),
    #                     Quaternion(-0.135, 0.350, 0.329, 0.866)),
    #                Pose(Point(0.339, 0.0116, 0.255),
    #                     Quaternion(0.245, 0.613, -0.202, 0.723))]

    target_poses = [Pose(Point(0.279, 0.176, 0.217),
                         Quaternion(0, 0, 0, 1)),
                    Pose(Point(0.239, 0.0116, 0.05),
                         Quaternion(0, 0.8509035, 0, 0.525322))]

    ik_solver = IK(ARM_BASE_LINK, GRIPPER_LINK)
    
    rospy.Subscriber('/joint_states', JointState, get_joint_state)

    arm_pub = rospy.Publisher(ROSTOPIC_SET_ARM_JOINT, JointState, queue_size=1)
    pan_pub = rospy.Publisher(ROSTOPIC_SET_PAN_JOINT, Float64, queue_size=1)
    tilt_pub = rospy.Publisher(ROSTOPIC_SET_TILT_JOINT, Float64, queue_size=1)
    gripper_open_pub = rospy.Publisher(ROSTOPIC_OPEN_GRIPPER, Empty, queue_size=1)
    gripper_close_pub = rospy.Publisher(ROSTOPIC_CLOSE_GRIPPER, Empty, queue_size=1)
    rospy.sleep(2)

    home_arm(arm_pub)
    close_gripper(gripper_close_pub)

    for pose in target_poses:

        rospy.loginfo('Commanding arm to pose {}'.format(pose))
	
        print(current_joint_state)
        if current_joint_state:
            target_joint = compute_ik(
            ik_solver, pose, current_joint_state)

        if target_joint:
            set_arm_joint(arm_pub, target_joint)
            rospy.sleep(5)

    open_gripper(gripper_open_pub)

    raw_input("Robot ready to close gripper. Press Enter to continue.")
    print("Robot moving. Please wait.")
    close_gripper(gripper_close_pub)
    rospy.sleep(4)

    home_arm(arm_pub)


if __name__ == "__main__":
    main()
