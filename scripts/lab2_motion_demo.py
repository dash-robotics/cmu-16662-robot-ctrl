#!/usr/bin/env python
"""
Example for demonstrating start and final joint positions for Lab 2.
"""

import sys

import numpy as np
import rospy

from sensor_msgs.msg import JointState


ROSTOPIC_SET_ARM_JOINT = '/goal_dynamixel_position'


def home_arm(pub):
    set_arm_joint(pub, np.zeros(5))
    rospy.sleep(5)

def set_arm_joint(pub, joint_target):
    joint_state = JointState()
    joint_state.position = tuple(joint_target)
    rospy.loginfo('Going to arm joint position: {}'.format(joint_state.position))
    pub.publish(joint_state)


def main():
    rospy.init_node('command_joints_example', anonymous=True)

    rad_from_deg = np.pi/180.
    start_joints_deg = [  0., 20., 50., -80., 0.]
    final_joints_deg = [135., 20., 50., -80., 0.]

    pub = rospy.Publisher(ROSTOPIC_SET_ARM_JOINT,
                          JointState, queue_size=1)
    rospy.sleep(2)
    
    raw_input("Robot ready to move to HOME POSITION. Press Enter to continue.")
    print("Robot moving. Please wait.")
    home_arm(pub)
    rospy.sleep(5)

    raw_input("Robot ready to move to START POSITION. Press Enter to continue.")
    print("Robot moving. Please wait.")
    set_arm_joint(pub, rad_from_deg*np.array(start_joints_deg))
    rospy.sleep(5)
    
    raw_input("Robot ready to move to FINAL POSITION. Press Enter to continue.")
    print("Robot moving. Please wait.")
    set_arm_joint(pub, rad_from_deg*np.array(final_joints_deg))
    rospy.sleep(10)

    raw_input("Robot ready to move to HOME POSITION. Press Enter to continue.")
    print("Robot moving. Please wait.")
    home_arm(pub)
    rospy.sleep(5)


if __name__ == "__main__":
    main()
