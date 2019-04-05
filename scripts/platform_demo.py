#!/usr/bin/env python
"""
Example for demonstrating arm, gripper, and camera motion.
"""

import sys

import numpy as np
import rospy

from std_msgs.msg import Float64
from std_msgs.msg import Empty

from sensor_msgs.msg import JointState


ROSTOPIC_SET_ARM_JOINT = '/goal_dynamixel_position'
ROSTOPIC_SET_PAN_JOINT = '/pan/command'
ROSTOPIC_SET_TILT_JOINT = '/tilt/command'
ROSTOPIC_OPEN_GRIPPER = '/gripper/open'
ROSTOPIC_CLOSE_GRIPPER = '/gripper/close'


def home_arm(pub):
    set_arm_joint(pub, np.zeros(5))
    rospy.sleep(5)

def set_arm_joint(pub, joint_target):
    joint_state = JointState()
    joint_state.position = tuple(joint_target)
    rospy.loginfo('Going to arm joint position (rad): {}'.format(joint_state.position))
    pub.publish(joint_state)

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

def open_gripper(pub):
    empty_msg = Empty()
    rospy.loginfo('Opening gripper')
    pub.publish(empty_msg)

def close_gripper(pub):
    empty_msg = Empty()
    rospy.loginfo('Closing gripper')
    pub.publish(empty_msg)


def main():
    rospy.init_node('platform_demo_example', anonymous=True)

    rad_from_deg = np.pi/180.
    target_arm_joints_rad = [
      [ 0.408, 0.721, -0.471, -1.4,  0.92],
      [-0.675, 0.   ,  0.23 ,  1. , -0.7 ]
    ]

    target_pan_joints_deg = [0., 15.0, -15.0, 0.]
    target_tilt_joints_deg = [0., 15.0, -15.0, 0.]

    arm_pub = rospy.Publisher(ROSTOPIC_SET_ARM_JOINT, JointState, queue_size=1)
    pan_pub = rospy.Publisher(ROSTOPIC_SET_PAN_JOINT, Float64, queue_size=1)
    tilt_pub = rospy.Publisher(ROSTOPIC_SET_TILT_JOINT, Float64, queue_size=1)
    gripper_open_pub = rospy.Publisher(ROSTOPIC_OPEN_GRIPPER, Empty, queue_size=1)
    gripper_close_pub = rospy.Publisher(ROSTOPIC_CLOSE_GRIPPER, Empty, queue_size=1)
    rospy.sleep(2)
    
    raw_input("Robot ready to move to HOME POSITION. Press Enter to continue.")
    print("Robot moving. Please wait.")
    home_arm(arm_pub)
    rospy.sleep(5)

    raw_input("Robot ready to demo arm motion. Press Enter to continue.")
    print("Robot moving. Please wait.")
    for arm_joints in target_arm_joints_rad:
        set_arm_joint(arm_pub, arm_joints)
        rospy.sleep(4)
    home_arm(arm_pub)

    raw_input("Robot ready to move camera pan motor. Press Enter to continue.")
    print("Robot moving. Please wait.")
    for pan_deg in target_pan_joints_deg:
        set_camera_pan(pan_pub, rad_from_deg*pan_deg)
        rospy.sleep(4)

    raw_input("Robot ready to move camera tilt motor. Press Enter to continue.")
    print("Robot moving. Please wait.")
    for tilt_deg in target_tilt_joints_deg:
        set_camera_tilt(tilt_pub, rad_from_deg*tilt_deg)
        rospy.sleep(4)

    raw_input("Robot ready to close gripper. Press Enter to continue.")
    print("Robot moving. Please wait.")
    close_gripper(gripper_close_pub)
    rospy.sleep(4)

    raw_input("Robot ready to open gripper. Press Enter to continue.")
    print("Robot moving. Please wait.")
    open_gripper(gripper_open_pub)
    rospy.sleep(4)


if __name__ == "__main__":
    main()
