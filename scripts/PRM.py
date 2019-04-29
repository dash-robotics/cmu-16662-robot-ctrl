import numpy as np 
import forward_kinematics_extrinsics as fk
import collision_checking as cc
from collections import defaultdict
import time
import pickle
import scipy.spatial.kdtree
import math
import heapq
import plotter

DOF = 5

# Node class for storing ID and state
class Node:
    def __init__(self, state, id):
        self.state = state
        self.id = id

    @staticmethod
    def distance(node1, node2):
        dist = np.zeros((1,node1.shape[0]))
        for i in range(node1.shape[0]):
            d = np.linalg.norm(node2[i] - node1[i])
            dist[0,i] = min(d, 360 - d)
        return np.linalg.norm(dist)

# Graph class for storing the PRM nodes and edges
class Graph:
    def __init__(self):
        self.adj_list = defaultdict(dict)
        self.nodes = defaultdict(np.float)
        self.ID = 0

    def addNode(self, sample):
        node = Node(sample, self.ID)
        self.nodes[self.ID] = sample
        self.ID += 1
        return node

    def deleteNode(self, node):
        connections = self.adj_list[node.id]
        del self.adj_list[node.id]
        for c in connections:
            del self.adj_list[c][node.id]

    def addEdge(self, new, nearest_node):
        self.adj_list[int(new)][nearest_node[0]] = nearest_node[1]
        self.adj_list[nearest_node[0]][int(new)] = nearest_node[1]

    def getVertices(self):
        pass

    def getEdges(self):
        pass

# PRM Class for creating a new graph and finding new paths using A-star
class ProbabilisticRoadMap:
    def __init__(self, obstacles_cuboids):
        self.constraint = np.tile(np.array([-179,180])[:,np.newaxis],(1,DOF))

        self.obstacles_cuboids = obstacles_cuboids
        ground_plane = np.array([[[0.0, 0.000, -0.10],[0, 0, 0],[2.0, 2.0, 0.01]]])
        self.obstacles_cuboids = np.append(self.obstacles_cuboids, ground_plane, axis=0)

        self.map = Graph()
        self.forward_kinematics = fk.forwardKinematics( base_link='bottom_plate', end_link='gripper_link')
        # self.anim = plotter.Plot()

    # Set constraint on the joint angles 
    def setConstraint(self,constraint_low, constraint_high):
        # Check if constraints are passed in correct format
        assert(self.constraint.shape[1] == constraint_high.shape[0]), \
            'The dimension of the high constraints is wrong'
        assert(self.constraint.shape[1] == constraint_low.shape[0]), \
            'The dimension of the low constraints is wrong'
        assert(np.all(constraint_low < constraint_high)), \
            'The low constraints are higher than high constraints'
        print('Constraints set')

        self.constraint[0,:] = constraint_low
        self.constraint[1,:] = constraint_high

    # check constraint for a joint state set
    def checkConstraint(self, state):
        # Check the statte against low and high constraints
        if np.all(state >= self.constraint[0,:]) and np.all(state <= self.constraint[1,:]):
            return True
        
        return False

    # sample a node and add in the graph if its in Cfree
    def sampleNode(self):
        # self.anim.clearPlot()
        samples = np.zeros((1,DOF))

        # Generate Sample within Constraints
        for i in range(samples.shape[1]):
            samples[:,i] = np.random.randint(self.constraint[0,i],self.constraint[1,i], size=1)

        # Check if the node is in collision with obstacles
        collision = self.checkPointCollision(samples)
        if collision == True:
            return None
        else: 
            # Add node to graph
            return self.map.addNode(samples)

    # Iterate over all nodes and find K nearest neighbours
    def findKNearestNeighbours(self, new_node, K): 
        # Stores node ID and distance to new_node
        dist = np.zeros((len(self.map.nodes),2))

        for i in range(len(self.map.nodes)):
            dist[i,0] = i
            dist[i,1] = Node.distance(new_node.state,self.map.nodes[i])

        # Sort the nodes based on distance
        dist = dist[dist[:,1].argsort()]

        # Send all nodes if  number of nodes are less than K
        if len(self.map.nodes) > K:
            return dist[1:K,:]
        else:
            return dist[1:,:]

    # Check if the sample is in collision with obstacles
    def checkPointCollision(self, sample):
        deg_to_rad = np.pi/180.
        
        np.insert(sample, 0, 0, axis=1)
        # Find arm cuboids properties in the current state
        arm_cuboids = self.forward_kinematics.getRotatedCuboid(deg_to_rad*sample)
        # self.Animation(arm_cuboids)
        

        for i in range(arm_cuboids.shape[0]):
            for j in range(self.obstacles_cuboids.shape[0]):
                collision = cc.collisionChecking(arm_cuboids[i,:,:], self.obstacles_cuboids[j,:,:]) 
                # print('arm', i, 'obstacle', j, collision)

                # If even one of the cuboids is in collsion, return true
                if collision is True:
                    return True

        # If no cuboids were in collision, return false
        return False

    # Draw cuboids in graph for visualization
    def Animation(self, arm_cuboids):
        # Plot all the arm cuboids
        for i in range(arm_cuboids.shape[0]):
            box = cc.Cuboid(arm_cuboids[i])
            self.anim.plotCuboid(box.getCorners(), i)

        # Plot all the collision cuboids
        for i in range(self.obstacles_cuboids.shape[0]):    
            box = cc.Cuboid(self.obstacles_cuboids[i])
            self.anim.plotCuboid(box.getCorners(), i)

        self.anim.showPlot(10)

    # Check if there are obstacles between new_node and its nearest neighbour
    def checkLineCollision(self, new_node, nearest_node):
        # The maximum angle by which the state is discretized 
        discrete_factor = 5.0

        # Check if the new node was connectable with any nearest neighbour
        connected = False

        # Iterate over nearest neighbours
        for i in range(nearest_node.shape[0]):
            # Find difference between the angles of the joint
            diff = self.map.nodes[nearest_node[i,0]] - new_node.state
            # Find out the max angle difference and calculate the number of steps to move
            steps = math.ceil(abs(diff).max()/discrete_factor)
            if steps == 0.0:
                continue

            for j in range(int(steps)+1):
                # Find the angles for the jth step
                sample = new_node.state + j*diff/steps

                # Check if the arm collides with obstacle at jth step
                collision = self.checkPointCollision(sample)
                if collision is True:
                    break

            # Add edge if no collision
            if collision is False:
                self.map.addEdge(new_node.id, nearest_node[i,:])
                connected = True
    
        return connected  

    def makeGraph(self, sim_time):
        print("Starting to Make Graph")

        # Number of nearest neighbour for connection
        K = 5

        constraint = np.ones((2,5))
        self.setConstraint(-90*constraint[0,:],90*constraint[1,:])

        start_time = time.time()

        # Keep adding nodes till the time is less the sim_time
        while time.time() - start_time < sim_time:
            new_node = self.sampleNode()
            if new_node is not None and len(self.map.nodes) > 1:
                nearest_node = self.findKNearestNeighbours(new_node, K)
                self.checkLineCollision(new_node, nearest_node)
        
        print(len(self.map.nodes))
        #print(self.map.adj_list)
        print('Graph Successfully Built')

        # Save graph for future use
        self.saveGraph()

    def saveGraph(self):
        # Save the adjecency list 
        with open('../data/prm_graph.pickle', 'wb') as outfile:
            pickle.dump(self.map.adj_list, outfile, protocol=pickle.HIGHEST_PROTOCOL)

        # Save the information about the nodes and their IDs
        with open('../data/prm_node.pickle', 'wb') as outfile:
            pickle.dump(self.map.nodes,outfile, protocol = pickle.HIGHEST_PROTOCOL)

    def makePlan(self, start, goal):
        # Number of nodes to connect the start and goal with
        K = 10

        # Check if start and goal satisfy constraints and aren't in collision
        assert self.checkConstraint(start), \
            'The given start node is unconstrained'
        assert self.checkConstraint(goal), \
            'The given goal node is unconstrained'
        assert self.checkPointCollision(start) == False, \
            'The given start node is in collision'
        assert self.checkPointCollision(goal) == False, \
            'The given goal node is in collision' 

        # Load saved PRM graph   
        self.loadGraph()

        # Connect start node in graph
        start_node = self.map.addNode(start)
        nearest_node = self.findKNearestNeighbours(start_node, K)

        assert self.checkLineCollision(start_node, nearest_node), \
            'The given start node was unable to find a connection'

        # Connect goal node in graph  
        goal_node = self.map.addNode(goal)
        nearest_node = self.findKNearestNeighbours(goal_node, K)

        assert self.checkLineCollision(goal_node, nearest_node), \
            'The given goal node was unable to find a connection'

        # Find the smallest path using A star
        path = self.Astar(start_node, goal_node)
        return path

    def Astar(self, start_node, goal_node):
        priority_list = []

        came_from = defaultdict(int)
        cost_so_far = defaultdict(int)
        came_from[start_node.id] = None
        cost_so_far[start_node.id] = 0
        heapq.heappush(priority_list, (Node.distance(goal_node.state, start_node.state), start_node.id))
        flag = 0

        while priority_list:
            # Find the node with the smallest f() cost
            item = heapq.heappop(priority_list)

            # Break if the current node is goal 
            if item[1] == goal_node.id:
                flag = 1
                break

            # Iterate over all the neighbouring nodes of the current node
            for next_node in self.map.adj_list[item[1]]:
                # Find the g() cost of the neighbouring node
                new_cost = self.map.adj_list[item[1]][next_node] + cost_so_far[item[1]]

                # If the node is unexplored or the g() cost is lesser than the previous estimate
                if next_node not in cost_so_far or cost_so_far[next_node] > new_cost:
                    # change the cost value to new cost
                    cost_so_far[next_node] = new_cost

                     # Calculate the f() cost and store in the priority queue
                    priority_cost = new_cost + Node.distance(self.map.nodes[next_node], goal_node.state)
                    heapq.heappush(priority_list, (priority_cost, next_node))

                    # Store the relation between current and nearest node for saving in the graph 
                    came_from[next_node] = item[1]
        
        # Find the path from goal to start
        next_id = goal_node.id
        path = [self.map.nodes[next_id]]

        # Find path only if goal node was reached
        while flag:

            # Terminate if the start node is reached
            if next_id == start_node.id:
                # Flip the array before returning it back
                path = np.flipud(np.squeeze(np.stack(path)))
                return path

            next_id = came_from[next_id]
            path.append(self.map.nodes[next_id])

        return None

    def loadGraph(self):
        with open('../data/prm_graph.pickle', 'rb') as infile:
            self.map.adj_list = pickle.load(infile)
    
        with open('../data/prm_node.pickle', 'rb') as infile:
            self.map.nodes = pickle.load(infile)

        # Set the ID of the graph to the number of nodes to make future nodes unique
        self.map.ID = len(self.map.nodes)


if __name__ == "__main__":
    print('Inside PRM Class')

    #obstacles_cuboids = np.load('../data/obstacle_cuboid.npy')
    obstacles_cuboids = np.array([[[-0.12, 0, 0.1025], [0, 0, 0], [0.08, 0.19, 0.205]],
                                  [[-0.11, 0, 0.305], [0, 0, 0], [0.07, 0.19, 0.20]],
                                  [[-0.06, 0.0, 0.455], [0, 0, 0], [0.1, 0.15, 0.1]]])
    prm = ProbabilisticRoadMap(obstacles_cuboids)
    # prm.makeGraph(120.0)

    start = np.array([[  0., 20., 50., -80., 0.]])
    goal= np.array([[90., 20., 50., -80., 0.]])
    path = prm.makePlan(start, goal)
    print(path)