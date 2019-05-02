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
# import camera_forward_kinematics
import tf
import PRM

GRIPPER_LINK = "gripper_link"
ARM_BASE_LINK = "bottom_plate"
MOVE_GROUP_NAME = 'arm'

ROSTOPIC_SET_ARM_JOINT = '/goal_dynamixel_position'
ROSTOPIC_SET_PAN_JOINT = '/pan/command'
ROSTOPIC_SET_TILT_JOINT = '/tilt/command'
ROSTOPIC_OPEN_GRIPPER = '/gripper/open'
ROSTOPIC_CLOSE_GRIPPER = '/gripper/close'

IK_POSITION_TOLERANCE = 0.01
IK_ORIENTATION_TOLERANCE = np.pi/9
current_joint_state = None
MIN_CLOSING_GAP = 0.002


def home_arm(pub):
    rospy.loginfo('Going to arm home pose')
    set_arm_joint(pub, np.zeros(5))
    rospy.sleep(5)

def set_camera_angles(pan_pub, tilt_pub, pan_rad, tilt_rad): #, ck):
    pan_msg = Float64()
    pan_msg.data = pan_rad
    rospy.loginfo('Going to camera pan: {} rad'.format(pan_rad))
    pan_pub.publish(pan_msg)

    tilt_msg = Float64()
    tilt_msg.data = tilt_rad
    rospy.loginfo('Going to camera tilt: {} rad'.format(tilt_rad))
    tilt_pub.publish(tilt_msg)

    # print("Camera Extrinsics", ck.cameraForwardKinematics(np.array([[pan_rad, tilt_rad, 0.]])))

def set_arm_joint(pub, joint_target):
    joint_state = JointState()
    joint_state.position = tuple(joint_target)
    pub.publish(joint_state)

def get_joint_state(data):
    global current_joint_state
    global current_gripper_state
    # if len(data.position) == 9:
    current_gripper_state = data.position[7:9]
    current_joint_state = data.position[0:5]

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
    global current_joint_state

    rad_from_deg = np.pi/180.
    deg_from_rad = 180./np.pi

    ik_solver = IK(ARM_BASE_LINK, GRIPPER_LINK)
    # ck = camera_forward_kinematics.camerakinematics()

    # Describing the publisher subscribers
    rospy.Subscriber('/joint_states', JointState, get_joint_state)

    arm_pub = rospy.Publisher(ROSTOPIC_SET_ARM_JOINT, JointState, queue_size=1)
    pan_pub = rospy.Publisher(ROSTOPIC_SET_PAN_JOINT, Float64, queue_size=1)
    tilt_pub = rospy.Publisher(ROSTOPIC_SET_TILT_JOINT, Float64, queue_size=1)
    gripper_open_pub = rospy.Publisher(ROSTOPIC_OPEN_GRIPPER, Empty, queue_size=1)
    gripper_close_pub = rospy.Publisher(ROSTOPIC_CLOSE_GRIPPER, Empty, queue_size=1)
    tf_listener = tf.TransformListener()
    rospy.sleep(2)

    # Homing of all servos
    home_joint = [0.004601942375302315, -0.4218447208404541, 1.6260197162628174, -0.1426602154970169, 0.010737866163253784]
    # home_joint = [0.0, 0.0, 1.22, -0.142, 0.0]
    set_arm_joint(arm_pub, home_joint)
    close_gripper(gripper_close_pub)
    set_camera_angles(pan_pub, tilt_pub, rad_from_deg*0., rad_from_deg*20.0) #, ck)

    rospy.sleep(10)


    while not rospy.is_shutdown():
        try:
            tf_listener.waitForTransform("/bottom_plate", "/icp_cuboid_frame", rospy.Time.now(), rospy.Duration(4.0))
            P, Q = tf_listener.lookupTransform("/bottom_plate", "/icp_cuboid_frame", rospy.Time(0))
            print("Box in bottom plate frame")
            print(P, Q)
            break
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
           print(e)

    O = tf.transformations.euler_from_quaternion(Q)
    Q = tf.transformations.quaternion_from_euler(0, np.pi/2, O[2])
    Q1 = tf.transformations.quaternion_from_euler(0, np.pi/2, O[2]+np.pi/2)

    poses = [Pose(Point(P[0], P[1], P[2]+0.20), Quaternion(Q[0], Q[1], Q[2], Q[3])),
             Pose(Point(P[0], P[1], P[2]+0.15), Quaternion(Q[0], Q[1], Q[2], Q[3])),
             Pose(Point(P[0], P[1], P[2]+0.25), Quaternion(Q[0], Q[1], Q[2], Q[3])),
             Pose(Point(P[0]-0.1, P[1]-0.2, P[2]+0.15), Quaternion(Q1[0], Q1[1], Q1[2], Q1[3]))]

    # Position, Orientation and Dimension
    obstacles_cuboids = np.array([[[-0.12, 0, 0.1025], [0, 0, 0], [0.08, 0.19, 0.205]],
                                  [[-0.11, 0, 0.305], [0, 0, 0], [0.07, 0.19, 0.20]],
                                  [[-0.06, 0.0, 0.455], [0, 0, 0], [0.1, 0.15, 0.1]],
                                  [[P[0], P[1], P[2]], [O[0], O[1], O[2]], [0.2, 0.03, 0.1]]])

    prm = PRM.ProbabilisticRoadMap(obstacles_cuboids)
    # prm.makeGraph(120.0)

    target_joint = None
    itr = 0
    home_arm(arm_pub)
    for pose in poses:
        print(current_joint_state)
        print("Reaching a joint state")
        if current_joint_state:
            target_joint = compute_ik(
            ik_solver, pose, current_joint_state)

        if target_joint:
            # print(np.array([current_joint_state]))
            # print(target_joint)

            path = prm.makePlan(np.array([current_joint_state])*deg_from_rad, np.array([target_joint])*deg_from_rad)
            # print(len(path))
            # for joint_angles in path:
            #     joint_angles = (joint_angles*rad_from_deg).tolist()
            #     set_arm_joint(arm_pub, np.array(joint_angles))
            #     rospy.sleep(5)
            set_arm_joint(arm_pub, target_joint)
            rospy.sleep(8)
            if itr == 0 or itr == 3:
                open_gripper(gripper_open_pub)
                rospy.sleep(4)
            if itr == 1:
                close_gripper(gripper_close_pub)
                rospy.sleep(4)
                # Use the following condition to detect if the object is grasped or not
                if(np.abs(current_gripper_state[0]) < MIN_CLOSING_GAP and np.abs(current_gripper_state[1] < MIN_CLOSING_GAP)):
                    state = "Not grabbed"
        rospy.sleep(4)
        itr += 1

    # raw_input("Robot ready to close gripper. Press Enter to continue."

    home_arm(arm_pub)
    open_gripper(gripper_open_pub)
    set_arm_joint(arm_pub, home_joint)


if __name__ == "__main__":
    main()

    # while not rospy.is_shutdown():
    #     try:
    #         tf_listener.waitForTransform("/camera_depth_frame", "/icp_cuboid_frame", rospy.Time.now(), rospy.Duration(4.0))
    #         position, quaternion = tf_listener.lookupTransform("/camera_depth_frame", "/icp_cuboid_frame", rospy.Time(0))
    #         print("Box in depth frame")
    #         print(position, quaternion)
    #         break
    #     except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
    #        print(e)
    #     #    return

    # # for pose in target_poses:
    # O = tf.transformations.euler_from_quaternion(quaternion)
    # R, T = ck.getTransformedPose(np.array([O[0], O[1], O[2]]), np.array([position[0], position[1], position[2]]))
    # print("Box in bottom plate frame" ,R, T)
    # q = tf.transformations.quaternion_from_euler(R[0], R[1], R[2])
    # poses = [Pose(Point(T[0], T[1], T[2]+0.1), Quaternion(q[0], q[1], q[2], q[3])),
    #         Pose(Point(T[0], T[1], T[2]), Quaternion(q[0], q[1], q[2], q[3]))]
    # rospy.loginfo('Commanding arm to pose {}'.format(poses))

    # q = tf.transformations.quaternion_from_euler(0, 90, 0)
    # poses = [Pose(Point(0.3, 0.000, 0.25), Quaternion(q[0], q[1], q[2], q[3])),
    #         Pose(Point(0.3, 0.000, 0.10), Quaternion(q[0], q[1], q[2], q[3]))]
