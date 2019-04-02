import numpy as np
import math 
import roslib; roslib.load_manifest('urdfdom_py')
import rospy
from urdf_parser_py.urdf import URDF

# Class for storing information about the robot and computing the forward kinematics 
class forwardKinematics:
        def __init__(self, base_link='arm_base_link', end_link='ar_tag', \
                path="/media/karmesh/DATA/CMU/Courses/Robot_Autonomy/Homework/hw3_release/hw3_release/code/locobot_description/urdf/locobot_description.urdf"):

                self.axis = None
                self.position = None
                # arm_cuboid_properties = np.load('../data/arm_cuboid_properties.npz')
                # self.arm_cuboid_origin = arm_cuboid_properties[arm_cuboid_properties.files[0]]
                # self.dimension = arm_cuboid_properties[arm_cuboid_properties.files[1]]
                robot = URDF.from_xml_file(path)
                self.urdfParser(robot, base_link, end_link)
                self.loadURDFinfo()

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
        def getJointForwardKinematics(self, joint_angles):
                Hij = np.identity(4)    
                H = []

                for i in range(joint_angles.shape[1]):
                        H21 = self.createTransformationMatrix(self.axis[i,:]*joint_angles[0,i],self.position[i,:])
                        Hij = np.matmul(Hij, H21)
                        H.append(Hij)

                H = np.stack(H)

                return H

        # Returns the Origin, Orientation and Dimension of the collision cuboids around the robot arm links
        # def getRotatedCuboid(self, joint_angles):
        #         H = self.getJointForwardKinematics(joint_angles)
        #         cuboid = np.zeros((7,3,3))

        #         # Cuboid information for links 1 to 5 which are connected to revolute joints
        #         for i in range(H.shape[0]):
        #                 # Origin of the cuboid (x,y,z)
        #                 cuboid[i,0,:] = np.matmul(H[i,:,:],self.arm_cuboid_origin[i,:].T)[0:3]

        #                 # Rotation angles of the cuboid (roll, pitch, yaw)
        #                 cuboid[i,1,:] = self.getEulerAngles(H[i,0:3,0:3])

        #                 # Dimension of the cuboid along (x,y,z)
        #                 cuboid[i,2,:] = self.dimension[i,:]

        #         # Cuboid information for the first finger
        #         cuboid[5,0,:] = np.matmul(H[4,:,:],self.arm_cuboid_origin[5,:].T)[0:3]
        #         cuboid[5,1,:] = self.getEulerAngles(H[4,0:3,0:3])
        #         cuboid[5,2,:] = self.dimension[5,:]

        #         # Cuboid information for the second finger
        #         cuboid[6,0,:] = np.matmul(H[4,:,:],self.arm_cuboid_origin[6,:].T)[0:3]
        #         cuboid[6,1,:] = self.getEulerAngles(H[4,0:3,0:3])
        #         cuboid[6,2,:] = self.dimension[6,:]

        #         return(cuboid)

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
                
                assert(forwardKinematics.isRotationMatrix(R)), \
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
                H[0:3,0:3] = forwardKinematics.createRotationMatrix(O)
                H[0:3,3] = T
                return H

        def urdfParser(self, robot, base_link, end_link):
                joint_list = robot.get_chain(base_link,end_link,links=False)
                self.position = np.zeros((6,3))
                self.axis = np.zeros((6,3))
                print(joint_list)
                for i in range(len(joint_list)):
                        self.position[i,:] = robot.joint_map[joint_list[i]].origin.xyz
                        self.axis[i,:] = robot.joint_map[joint_list[i]].axis
                
                np.savez('../data/urdfinfo.npz', position=self.position, axis=self.axis)

        def loadURDFinfo(self):
                file = np.load('../data/urdfinfo.npz')
                self.position = file[file.files[0]]
                self.axis = file[file.files[1]]
                # print(self.position)
                # print(self.axis)

if __name__ == "__main__":
	# calibration_info = np.load('../data/calibration/calibration_info.npz')
	# joint_angles = calibration_info[calibration_info.files[3]][0].reshape((1,5))
	# fk = forwardKinematics()
        # M = fk.getJointForwardKinematics(joint_angles)
        X = np.array([[ 0., 20., 50., -80., 45., 0.]])
        fk = forwardKinematics()
        M = fk.getJointForwardKinematics(X*np.pi/180)
        for i in range(M.shape[0]):
                print(np.matmul(np.linalg.inv(M[0,:,:]),M[i,:,:]))
