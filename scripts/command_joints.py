#!/usr/bin/env python
"""
Example for commanding joints
"""

import sys

import numpy as np
import rospy
from sensor_msgs.msg import JointState

ROSTOPIC_SET_ARM_JOINT = '/goal_dynamixel_position'


def home_arm(pub):
    rospy.loginfo('Going to arm home pose')
    set_arm_joint(pub, np.zeros(5))
    rospy.sleep(5)


def set_arm_joint(pub, joint_target):
    joint_state = JointState()
    joint_state.position = tuple(joint_target)
    pub.publish(joint_state)


def main():
    rospy.init_node('command_joints_example', anonymous=True)

    target_joints = [
        [0.408, 0.721, -0.471, -1.4, 0.920],
        [-0.675, 0, 0.23, 1, -0.70]
    ]

    pub = rospy.Publisher(ROSTOPIC_SET_ARM_JOINT,
                          JointState, queue_size=1)
    rospy.sleep(2)
    home_arm(pub)

    for joint in target_joints:
      set_arm_joint(pub, joint)
      rospy.sleep(4)

    home_arm(pub)


if __name__ == "__main__":
    main()