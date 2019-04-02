#!/usr/bin/env python
"""
Example for demonstrating start and final joint positions for Lab 2.
"""

import sys

import numpy as np
import rospy

from sensor_msgs.msg import JointState
import PRM
import forward_kinematics_extrinsics


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

    pub = rospy.Publisher(ROSTOPIC_SET_ARM_JOINT,
                          JointState, queue_size=1)
    rospy.sleep(2)

    rad_from_deg = np.pi/180.

    raw_input("Robot ready to move to HOME POSITION. Press Enter to continue.")
    print("Robot moving. Please wait.")
    home_arm(pub)

    X = np.array([[ 0., 20., 50., -80., 45.]])
    fk = forward_kinematics_extrinsics.forwardKinematics('arm_base_link', 'ar_tag')
    M = fk.getJointForwardKinematics(X[0,0:,4]*rad_from_deg)
    for i in range(M.shape[0]):
    	print(np.matmul(np.linalg.inv(M[0,:,:]),M[i,:,:]))

    raw_input("Robot ready to move to HOME POSITION. Press Enter to continue.")
    print("Robot moving. Please wait.")
    print(X)
    set_arm_joint(pub, X.reshape((5))*rad_from_deg)
    rospy.sleep(5)

    raw_input("Robot ready to move to HOME POSITION. Press Enter to continue.")

    start = np.array([[  0., 20., 50., -80., 0.]])
    goal = np.array([[85., 20., 50., -80., 0.]])
    
    # Create PRM variable and get path from start to goal
    obstacles_cuboids = np.array([[[0.0604, 0.1854, 0.214], [0, 0, 0], [0.1, 0.125, 0.225]], \
                                  [[0.0604, 0.1854, 0.349], [0, 0, 0], [0.125, 0.225, 0.1]]])

    prm = PRM.ProbabilisticRoadMap(obstacles_cuboids)

    path = prm.makePlan(start, goal)

    # # Send nodes of the path as commands to controller 
    for joint_angles in path:
        print("Robot moving. Please wait.")
        joint_angles = (joint_angles*rad_from_deg).tolist()
        set_arm_joint(pub, np.array(joint_angles))
        print(joint_angles)
        rospy.sleep(4)


    path = prm.makePlan(goal, start)

    # # Send nodes of the path as commands to controller 
    for joint_angles in path:
        print("Robot moving. Please wait.")
        joint_angles = (joint_angles*rad_from_deg).tolist()
        set_arm_joint(pub, np.array(joint_angles))
        print(joint_angles)
        rospy.sleep(4)

    # raw_input("Robot ready to move to HOME POSITION. Press Enter to continue.")
    # print("Robot moving. Please wait.")
    # home_arm(pub)
    # 

    # raw_input("Robot ready to move to START POSITION. Press Enter to continue.")
    # print("Robot moving. Please wait.")
    # set_arm_joint(pub, rad_from_deg*np.array(start_joints_deg))
    # rospy.sleep(5)
    
    # raw_input("Robot ready to move to FINAL POSITION. Press Enter to continue.")
    # print("Robot moving. Please wait.")
    # set_arm_joint(pub, rad_from_deg*np.array(final_joints_deg))
    # rospy.sleep(10)

    # raw_input("Robot ready to move to HOME POSITION. Press Enter to continue.")
    # print("Robot moving. Please wait.")
    # home_arm(pub)
    # rospy.sleep(5)


if __name__ == "__main__":
    main()
