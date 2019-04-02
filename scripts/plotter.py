import numpy as np
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection, Line3DCollection
import matplotlib.pyplot as plt

class Plot:
    def __init__(self):
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.colors = ['b', 'g', 'r', 'c', 'm', 'y', 'k', 'w']
    
    def showPlot(self, pause_time = 0.1):
        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Y')
        self.ax.set_zlabel('Z')
        plt.pause(pause_time)

    def plotCuboid(self, p, boxnumber=0):
        # Plot corners of cuboid
        self.ax.scatter3D(p[:, 0], p[:, 1], p[:, 2])

        # List of the 6 sides of cuboid
        sides = [[p[0],p[1],p[3],p[2]],
                [p[4],p[5],p[7],p[6]], 
                [p[4],p[5],p[1],p[0]], 
                [p[2],p[3],p[7],p[6]], 
                [p[1],p[5],p[7],p[3]],
                [p[0],p[4],p[6],p[2]]]

        # plot sides with different color of each cuboid
        boxnumber = int(boxnumber%8)
        self.ax.add_collection3d(Poly3DCollection(sides, 
        facecolors=self.colors[boxnumber], linewidths=1, edgecolors=self.colors[7-boxnumber], alpha=.50))

    def clearPlot(self):
        self.ax.clear()

if __name__ == "__main__":
    pt = Plot()
    p = np.array([[[ 1, 1, 1],
                  [-1, 1, 1],
                  [ 1,-1, 1],
                  [-1,-1, 1],
                  [ 1, 1,-1],
                  [-1, 1,-1],
                  [ 1,-1,-1],
                  [-1,-1,-1]],
                  [[ 2, 2, 2],
                  [-2, 2, 2],
                  [ 2,-2, 2],
                  [-2,-2, 2],
                  [ 2, 2,-2],
                  [-2, 2,-2],
                  [ 2,-2,-2],
                  [-2,-2,-2]]])
    pt.plotCuboid(p[0],1)
    pt.plotCuboid(p[1],2)
    pt.showPlot(10)
    #pt.clearPlot()