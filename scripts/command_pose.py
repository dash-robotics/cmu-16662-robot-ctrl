#!/usr/bin/env python
"""
Example for commanding robot pose
"""

import sys

import numpy as np
import rospy
from geometry_msgs.msg import Pose, Point, Quaternion
from sensor_msgs.msg import JointState
from trac_ik_python.trac_ik import IK

GRIPPER_LINK = "gripper_link"
ARM_BASE_LINK = "arm_base_link"
ROSTOPIC_SET_ARM_JOINT = '/goal_dynamixel_position'
ROSTOPIC_GET_ARM_JOINT = '/joint_states'
IK_POSITION_TOLERANCE = 0.01
IK_ORIENTATION_TOLERANCE = np.pi
joint_state = None

def home_arm(pub):
    rospy.loginfo('Going to arm home pose')
    set_arm_joint(pub, np.zeros(5))
    rospy.sleep(5)


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


def set_arm_joint(pub, joint_target):
    joint_state = JointState()
    joint_state.position = tuple(joint_target)
    pub.publish(joint_state)

def callback(msg):
  global joint_state
  joint_state = msg.position

def main():
    rospy.init_node('command_pose_example', anonymous=True)

    target_poses = [Pose(Point(0.279, 0.176, 0.217),
                         Quaternion(-0.135, 0.350, 0.329, 0.866)),
                    Pose(Point(0.339, 0.0116, 0.255),
                         Quaternion(0.245, 0.613, -0.202, 0.723))]

    ik_solver = IK(ARM_BASE_LINK, GRIPPER_LINK)
    pub = rospy.Publisher(ROSTOPIC_SET_ARM_JOINT,
                          JointState, queue_size=1)
    sub = rospy.Subscriber(ROSTOPIC_GET_ARM_JOINT, JointState, callback)
    rospy.sleep(2)
    home_arm(pub)

    for pose in target_poses:

        rospy.loginfo('Commanding arm to pose {}'.format(pose))
        target_joint = compute_ik(
            ik_solver, pose, joint_state[:5])

        if target_joint:
            set_arm_joint(pub, target_joint)
            rospy.sleep(5)

    home_arm(pub)


if __name__ == "__main__":
    main()