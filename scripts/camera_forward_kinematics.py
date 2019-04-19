import numpy as np
import math 
import roslib; roslib.load_manifest('urdfdom_py')
import rospy
from urdf_parser_py.urdf import URDF

# Class for storing information about the robot and computing the forward kinematics 
class camerakinematics:
        def __init__(self, base_link='bottom_plate', end_link='camera_link', \
                path="/media/karmesh/DATA/CMU/Courses/Robot_Autonomy/Homework/hw3_release/hw3_release/code/locobot_description/urdf/locobot_description.urdf"):

                self.axis = None
                self.position = None
                self.camera_link_to_camera_depth_frame = self.createTransformationMatrix([0, 0, 0], [0, 0.0149, 0])
                robot = URDF.from_xml_file(path)
                self.urdfParser(robot, base_link, end_link)
                self.loadURDFinfo()
                self.extrinsics_H = None

        # Creates Rotation Matrix given roll, pitch and yaw
        @staticmethod
        def createRotationMatrix(orientation):
                r,p,y = orientation

                R_x = np.array([[1,         0,                  0    ],
                                [0,         math.cos(r), -math.sin(r)],
                                [0,         math.sin(r),  math.cos(r)]]) 
                        
                R_y = np.array([[math.cos(p),    0,      math.sin(p)],
                                [0,              1,      0          ],
                                [-math.sin(p),   0,      math.cos(p)]])
                        
                R_z = np.array([[math.cos(y),    -math.sin(y),    0],
                                [math.sin(y),    math.cos(y),     0],
                                [0,              0,               1]])
                                        
                R = np.matmul(R_z, np.matmul( R_y, R_x ))

                return R

        # Returns the Homogeneous Transformation Matrix between joint i and ground, where i = [1,2,3,4,5,6,7]
        def cameraForwardKinematics(self, joint_angles):
                Hij = np.identity(4)    
                H = []

                for i in range(joint_angles.shape[1]):
                        H21 = self.createTransformationMatrix(self.axis[i,:]*joint_angles[0,i],self.position[i,:])
                        Hij = np.matmul(Hij, H21)
                        H.append(Hij)
                        print(Hij)

                # H = np.stack(H)
                self.extrinsics_H = np.matmul(Hij, self.camera_link_to_camera_depth_frame)
                return self.extrinsics_H

        def getTransformedPose(self, R, T):
                H = self.createTransformationMatrix(R, T)
                new_H = np.matmul(self.extrinsics_H, H)
                return self.getEulerAngles(new_H[0:3, 0:3]), H[0:3, 3]

        # Checks if a matrix is a valid rotation matrix.
        @staticmethod
        def isRotationMatrix(R):
                Rt = np.transpose(R)
                shouldBeIdentity = np.dot(Rt, R)
                I = np.identity(3, dtype = R.dtype)
                n = np.linalg.norm(I - shouldBeIdentity)
                return n < 1e-6
 
        # Calculates rotation matrix to euler angles
        @staticmethod
        def getEulerAngles(R):
                
                assert(camerakinematics.isRotationMatrix(R)), \
                        'Not a rotation matrix'
                
                sy = math.sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])
                
                singular = sy < 1e-6
                
                if  not singular:
                        x = math.atan2(R[2,1], R[2,2])
                        y = math.atan2(-R[2,0], sy)
                        z = math.atan2(R[1,0], R[0,0])
                else :
                        x = math.atan2(-R[1,2], R[1,1])
                        y = math.atan2(-R[2,0], sy)
                        z = 0
                
                return np.array([x, y, z])

        # Creates Homogeneous Transformation Matrix from orientation and translation information
        @staticmethod
        def createTransformationMatrix(O,T):
                H = np.identity(4)
                H[0:3,0:3] = camerakinematics.createRotationMatrix(O)
                H[0:3,3] = T
                return H

        def urdfParser(self, robot, base_link, end_link):
                joint_list = robot.get_chain(base_link,end_link,links=False)
                self.position = np.zeros((len(joint_list),3))
                self.axis = np.zeros((len(joint_list),3))
                print(joint_list)
                for i in range(len(joint_list)):
                        self.position[i,:] = robot.joint_map[joint_list[i]].origin.xyz
                        self.axis[i,:] = robot.joint_map[joint_list[i]].axis
                
                np.savez('../data/cameraurdfinfo.npz', position=self.position, axis=self.axis)

        def loadURDFinfo(self):
                file = np.load('../data/cameraurdfinfo.npz')
                self.position = file[file.files[0]]
                self.axis = file[file.files[1]]
                # print(self.position)
                # print(self.axis)

if __name__ == "__main__":
	# calibration_info = np.load('../data/calibration/calibration_info.npz')
	# joint_angles = calibration_info[calibration_info.files[3]][0].reshape((1,5))
	# fk = forwardKinematics()
        # M = fk.getJointForwardKinematics(joint_angles)
    X = np.array([[90., 0., 0.]])
    ck = camerakinematics()
    M = ck.getCameraForwardKinematics(X*np.pi/180)
    # for i in range(M.shape[0]):
    #     print(np.matmul(np.linalg.inv(M[0,:,:]),M[i,:,:]))
    print(M)
