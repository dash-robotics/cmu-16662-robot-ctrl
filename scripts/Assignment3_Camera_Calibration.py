#!/usr/bin/env python
"""
Example for demonstrating start and final joint positions for Lab 2.
"""
import cv2
import rospy
import numpy as np
from scipy import linalg

from sensor_msgs.msg import (JointState,
                             Image,
                             CameraInfo)
from cv_bridge import CvBridge, CvBridgeError
from ar_track_alvar_msgs.msg import AlvarMarkers
import forward_kinematics_extrinsics

ROSTOPIC_SET_ARM_JOINT = '/goal_dynamixel_position'
#flag = 0

def home_arm(pub):
    set_arm_joint(pub, np.zeros(5))
    rospy.sleep(5)

def set_arm_joint(pub, joint_target):
    joint_state = JointState()
    joint_state.position = tuple(joint_target)
    rospy.loginfo('Going to arm joint position: {}'.format(joint_state.position))
    pub.publish(joint_state)

def generatesample():
    sample = np.zeros(5)
    # Generate Sample within Constraints
    for i in range(sample.shape[0]):
        if i == 3:
            sample[i] = np.random.randint(-10, 10, size=1) - 90
        elif i == 1:
            sample[i] = np.random.randint(-5, 30, size=1)
        else:
            sample[i] = np.random.randint(-30, 30, size=1)
    print(sample)
    return sample

def main(noiter=15):
    rospy.init_node('camera_calibration', anonymous=True)

    bridge = CvBridge()
    pub = rospy.Publisher(ROSTOPIC_SET_ARM_JOINT, JointState, queue_size=1)
    rospy.sleep(2)
    Calib = rospy.wait_for_message("/camera/color/camera_info", CameraInfo)
    K_matrix = np.array(Calib.K)
    fk = forward_kinematics_extrinsics.forwardKinematics('arm_base_link', 'ar_tag')

    positions = np.zeros((noiter, 3))
    orientations = np.zeros((noiter, 4))
    joint_angles = np.zeros((noiter, 5))
    end_effector_positions = np.zeros((noiter, 4, 4))
    rad_from_deg = np.pi/180.

    # Send to Home Position
    raw_input("Robot ready to move to HOME POSITION. Press Enter to continue.")
    print("Robot moving. Please wait.")
    home_arm(pub)

    def imgcallback(i):
        data = rospy.wait_for_message("/camera/color/image_raw", Image, timeout=5)
        cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
        cv2.imwrite('../data/calibration/image_%d.png'%i, cv_image)
        return 0

    def artagcallback(i):
        artag = rospy.wait_for_message("ar_pose_marker", AlvarMarkers, timeout=5)
        if len(artag.markers) == 0:
            raise Exception("AR tag not found")
        else:
            pos = artag.markers[0].pose.pose.position
            q = artag.markers[0].pose.pose.orientation
            positions[i, :] = np.array([pos.x, pos.y, pos.z])
            orientations[i, :] = np.array([q.x, q.y, q.z, q.w])
            return 0

    # # Send nodes of the path as commands to controller
    count = 0
    while count < noiter:

        joint_angle = generatesample()
        joint_angles[count, :] = joint_angle
        print(joint_angles)
        joint_angle = (joint_angle*rad_from_deg).tolist()
        set_arm_joint(pub, np.array(joint_angle))

        joint_angle = np.append(joint_angle, 0)

        M = fk.getJointForwardKinematics(joint_angle.reshape((1,6)))
        end_effector_positions[count, :, :] = np.matmul(np.linalg.inv(M[0,:,:]),M[5,:,:])

        rospy.sleep(4)

        try:
            artagcallback(count)
        except (rospy.ROSException, Exception) as e:
            continue


        try:
            imgcallback(count)
        except (rospy.ROSException,CvBridgeError) as e:
            continue

        count += 1

    np.savez("../data/calibration/calibration_info.npz", position=positions, orientation=orientations, camerainfo=K_matrix, \
                                                         joint_angles=joint_angles, forward_kinematics=end_effector_positions)

    R = calculate_extrinsic(positions, K_matrix, end_effector_positions)
    print(R)


def calculate_extrinsic(positions, K_matrix, end_effector_positions):
    p = get_p(K_matrix, positions)
    P = get_P(end_effector_positions)
    M = get_projection_matrix(p, P)
    K,R = linalg.rq(M[:,0:3])
    print(K/K[2,2])
    T = np.matmul(np.linalg.inv(K),M[:,3])
    H = np.vstack((np.hstack((R,T.reshape((3,1)))),np.array([[0,0,0,1]])))
    return H

def get_projection_matrix(p, P):
    num_points = p.shape[0]
    xi = P[:,0].reshape((num_points,1))
    yi = P[:,1].reshape((num_points,1))
    zi = P[:,2].reshape((num_points,1))
    ui = p[:,0].reshape((num_points,1))
    vi = p[:,1].reshape((num_points,1))
    ones = np.ones((num_points, 1))
    zeros = np.zeros((num_points, 1))
    A = np.zeros((num_points*2,12))
    A[0:2*num_points:2,:] = np.hstack((xi, yi, zi, ones, zeros, zeros, zeros, zeros, -ui*xi, -ui*yi, -ui*zi, -ui))
    A[1:2*num_points:2,:] = np.hstack((zeros, zeros, zeros, zeros, xi, yi, zi, ones, -vi*xi, -vi*yi, -vi*zi, -vi))
    w,v = np.linalg.eigh(np.matmul(A.T, A))
    M = v[:,0].reshape((3,4))
    return M


def get_p(K_matrix, position):
    intrinsic = K_matrix.reshape((3,3))
    K_3_4 = np.hstack((intrinsic, np.array([[0],[0],[0]])))
    homogeneous_3D_point = np.hstack((position, np.ones((position.shape[0], 1))))
    homogeneous_pixel_coordinate = np.matmul(K_3_4,homogeneous_3D_point.T)
    homogeneous_pixel_coordinate = homogeneous_pixel_coordinate/homogeneous_pixel_coordinate[2,:]
    return homogeneous_pixel_coordinate[0:2,:].T


def get_P(end_effector_position):
    point_3D_world_frame = end_effector_position[:,0:3,3]
    return point_3D_world_frame

if __name__ == "__main__":
    main(30)