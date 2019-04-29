import numpy as np 
from sklearn.preprocessing import normalize
import forward_kinematics_extrinsics as fk

# Cuboid Class for creating cuboids from Origin, Orientation and Dimension
class Cuboid:

    def __init__(self, obj):
        self.origin = obj[0,:]
        self.orientation = obj[1,:]
        self.dimension = obj[2,:]

        self.corners = np.zeros((8,3))
        self.axes = np.zeros((3,3))

        self.R = np.zeros((3,3))
        self.createCuboid()

    # Finds the corners and axes 
    def createCuboid(self):
        # Creates Rotation Matrix for cuboid 
        self.R = fk.forwardKinematics.createRotationMatrix(self.orientation)

        self.findCorners()

        self.findAxes()        


    def findCorners(self):
        # Corners of a cuboid of length one and orientation of zero along x,y and z
        direction = np.array([[ 0.5, 0.5, 0.5],[-0.5, 0.5, 0.5], \
                              [ 0.5,-0.5, 0.5],[-0.5,-0.5, 0.5], \
                              [ 0.5, 0.5,-0.5],[-0.5, 0.5,-0.5], \
                              [ 0.5,-0.5,-0.5],[-0.5,-0.5,-0.5]])

        # Dimension along x,y and z
        D = np.tile(self.dimension[np.newaxis,:], (8, 1))

        # Cuboid scaled according to dimensions
        direction = direction*D

        # Origin of the cuboid
        O = np.tile(self.origin[np.newaxis,:], (8,1))

        # Corners after rotation by R and translation by O
        self.corners = np.matmul(self.R, (direction).T).T + O

    def findAxes(self):
        # Axis of the cuboid before rotation
        direction = np.array([[1,0,0],[0,1,0],[0,0,1]])

        # Rotation and normalization
        self.axes = np.matmul(self.R, direction).T
        self.axes = normalize(self.axes, axis=1)
    
    def getAxes(self):
        return self.axes

    def getCorners(self):
        return self.corners

    # Projection of the cuboid corners on an axis
    def project(self, axis):
        return np.matmul(self.corners, axis)

# Finds of the 15 collision axes from the normals of the two cuboids
def createCollisionAxis(ax1, ax2):  
    axes = ax1
    axes = np.vstack((axes, ax2))

    for i in range(ax1.shape[0]):
        for j in range(ax2.shape[0]):
            # Cross product of axis of cuboid 1 with cuboid 2
            axis = normalize( np.cross(ax1[i, :], ax2[j, :]).reshape(1,-1) )
            axes = np.vstack((axes, axis))
    return axes

# Finds the projecttion of the corners of the 2 cuboids on the Collision Axis
def checkProjection(box1, box2, axes):
    for axis in axes:
        proj1 = box1.project(axis)      
        proj2 = box2.project(axis)
        p1_min = proj1.min()
        p1_max = proj1.max()
        p2_min = proj2.min()
        p2_max = proj2.max()
    
        # Finds if there is a collision according to a particular axis
        if (p1_max < p2_min and p1_min < p2_min) or \
            (p2_max < p1_min and p2_min < p1_min):
                # print(axis)
                # print(box1.corners)
                # print(box2.corners)
                return False

    return True

# Main function with calls all the other collsion detection functions 
def collisionChecking(obj1, obj2):
    box1 = Cuboid(obj1)
    box2 = Cuboid(obj2)
    ax1 = box1.getAxes()
    ax2 = box2.getAxes()
    axes = createCollisionAxis(ax1, ax2)
    collision = checkProjection(box1, box2, axes)
    return collision

if __name__ == "__main__":
    print("Doing Collision Checking")

    ref_cube = np.array([[0, 0, 0], [0, 0, 0], [3, 1, 2]])

    test_set = np.array([[[ 0.0, 1.0, 0.0], [ 0.0, 0.0, 0.0], [ 0.8, 0.8, 0.8]], \
                         [[ 1.5,-1.5, 0.0], [ 1.0, 0.0, 1.5], [ 1.0, 3.0, 3.0]], \
                         [[ 0.0, 0.0,-1.0], [ 0.0, 0.0, 0.0], [ 2.0, 3.0, 1.0]], \
                         [[ 3.0, 0.0, 0.0], [ 0.0, 0.0, 0.0], [ 3.0, 1.0, 1.0]], \
                         [[-1.0, 0.0,-2.0], [ 0.5, 0.0, 0.4], [ 2.0, 0.7, 2.0]], \
                         [[ 1.8, 0.5, 1.5], [-0.2, 0.5, 0.0], [ 1.0, 3.0, 1.0]], \
                         [[ 0.0,-1.2, 0.4], [0.0,0.785,0.785],[ 1.0, 1.0, 1.0]], \
                         [[-0.8, 0.0,-0.5], [ 0.0, 0.0, 0.2], [ 1.0, 0.5, 0.5]]])
             
    for i in range(test_set.shape[0]):
            collision = collisionChecking(ref_cube, test_set[i,:,:])
            print(i+1,collision)
