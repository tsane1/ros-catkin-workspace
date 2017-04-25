#!/usr/bin/env python

""" 
ROS Server calculating the path between two points on a map using the A* algorithm

Author:
    - Tanuj Sane
    - Matthew Lamare
    - Matthew Piazza

Date: 4/10/17
"""

import math
import rospy, tf
from nav_msgs.msg import GridCells, Path
from geometry_msgs.msg import Point, PoseStamped, Pose
from tsane_mmlamare_mwpiazza_final.srv import *

Z_AXIS = (0, 0, 1)
TOLERANCE = .5 # meters

# A* Map Point 
class StarNode():
    def __init__(self, x, y, isWall, isUnknown, row, col, heuristicCost):
        self.x = x
        self.y = y            
        self.isWall = isWall
        self.isUnknown = isUnknown
        self.row = row
        self.col = col    
        self.knownCost = -1
        self.predictedCost = -1
        self.heuristicCost = heuristicCost

    def __repr__(self):
        return str(self.x) + ',' + str(self.y) + ': WALL' if self.isWall else ''

# ROS node 
class AStarServiceServer():
    # set up server  
    def __init__(self):                
        rospy.init_node('a_star_server_apo')
        s = rospy.Service('a_star', AStar, self.handleAStar)
        self.frameID = "map"
        rospy.spin()                    

    # run A* algorithm and return fields
    def handleAStar(self, msg):
        waypointArray = []
        self.readOccupancyGridMaps(msg.map, msg.costMap)
        goal = self.decideGoal(msg.start.position)
        print(goal)

        if self.distance(msg.start.position, goal.position) > TOLERANCE:       
            self.frameID = msg.frameID.data      
            waypointArray = self.aStar(msg.start.position.x, msg.start.position.y, msg.goal.position.x, msg.goal.position.y)
        else:
            print("A* Server: Goal Reached!")
        waypoints = self.calculateWaypoints(waypointArray)        
        return AStarResponse(waypoints)

    def decideGoal(self, startPosn):
        startNode = self.findNode(startPosn)
        print('start:', startNode)

        queue = [startNode]; goalPose = None
        while goalPose == None:
            currNode = queue.pop(0)
            for neighbor in self.getNeighbors(currNode):
                if neighbor.isUnknown:
                    goalPose = Pose()
                    goalPose.position.x = currNode.x
                    goalPose.position.y = currNode.y
                else:
                    queue.append(currNode)
        
        print('goal:', goalPose)
        return goalPose

    """
    WHY WON'T THIS JUST FUCKING WORK
    """
    def findNode(self, posn):
        for row in range(len(self.starMap)):
            for col in range(len(self.starMap[row])):
                x = (col*self.resolution) + posn.x + self.resolution/2.0
                y = (row*self.resolution) + posn.y + self.resolution/2.0

                test = self.starMap[row][col]
                if test.x == x and y == posn.y: return test

        print('nope fuck you')
        return None


    # estimates distance between two nodes
    def distance(self, currentPoint, goalPoint):     
        diffX = abs(currentPoint.x - goalPoint.x)
        diffY = abs(currentPoint.y - goalPoint.y)                
        return math.hypot(diffX, diffY) 

    # convert occupancy grid from RViz to 2D matrix map of StarNodes
    def readOccupancyGridMaps(self, wallMap, costMap):        
        mapRows = wallMap.info.height
        mapColumns = wallMap.info.width
        self.resolution = wallMap.info.resolution        
        self.origin = wallMap.info.origin          
        
        # create map of nodes
        if costMap.data:
            grid = [[(wallMap.data[(row*mapColumns) + col], costMap.data[(row*mapColumns) + col]) for col in range(mapColumns)] for row in range(mapRows)]
        else:
            # without costmap, all cells are equally costly
            grid = [[(wallMap.data[(row*mapColumns) + col], 1) for col in range(mapColumns)] for row in range(mapRows)]
        self.starMap = [[None for col in range(len(grid[row]))] for row in range(len(grid))]  
        for row in range(len(grid)):
            for col in range(len(grid[row])):
                x = (col*self.resolution) + self.origin.position.x + self.resolution/2.0
                y = (row*self.resolution) + self.origin.position.y + self.resolution/2.0
                isWall = grid[row][col][0] == 100
                isUnknown = grid[row][col][0] == -1            
                cost = grid[row][col][1]
                self.starMap[row][col] = StarNode(x, y, isWall, isUnknown, row, col, cost)

    # calculates best path between the set start and end nodes
    def aStar(self, startX, startY, goalX, goalY):
        startNode = self.getNodeFromXY(startX, startY)
        goalNode = self.getNodeFromXY(goalX, goalY)        
        if startNode and goalNode:
            self.visitedNodes = [] # list of evaluated nodes
            self.frontierNodes = [startNode] # list of nodes needing to be evaluated
            previousStepTo = {} # linked list of most efficient previous steps
            startNode.knownCost = 0
            startNode.predictedCost = self.heuristic(startNode, goalNode)

            while self.frontierNodes != []:
                # check for finished
                currentNode = self.getMinimumNode(startNode.isWall) 
                if currentNode == goalNode:        
                    return self.buildPath(previousStepTo, currentNode)

                # evaluate possible steps from current node
                self.frontierNodes.remove(currentNode)
                self.visitedNodes.append(currentNode)
                for neighborNode in self.getNeighbors(currentNode):
                    # skip neighbor if already visited, otherwise it is a frontier cell to check
                    if neighborNode in self.visitedNodes:
                        continue

                    # check if neighbor is new cell and an optimal path
                    neighborKnownCost = currentNode.knownCost + self.heuristic(currentNode, neighborNode)
                    """
                    If the start node is a wall, then the frontier expansion with getMinimumNode is circular because
                    all surrounding nodes are appended in a BFS manner. The end goal is overwritten to be the closest
                    open cell. 
                    Otherwise if the start node is not a wall, walls are not allowed to be in the frontier and 
                    the A* frontier expansion is directed towards the goal.
                    """
                    if neighborNode not in self.frontierNodes and (not neighborNode.isWall or startNode.isWall):
                        self.frontierNodes.append(neighborNode)
                        if startNode.isWall and not neighborNode.isWall:
                            goalNode = neighborNode
                            self.frontierNodes = [neighborNode]
                    elif neighborKnownCost >= neighborNode.knownCost and neighborNode.knownCost != -1:
                        continue

                    # update neighbor node with new optimal costs
                    previousStepTo[neighborNode] = currentNode
                    neighborNode.knownCost = neighborKnownCost
                    neighborNode.predictedCost = neighborNode.knownCost + self.heuristic(neighborNode, goalNode)

        print("A* Server: Could not find a path :( ")
        return []

     # finds center of grid cell containing coordinate
    def getNodeFromXY(self, x, y):
        for row in range(len(self.starMap)):
            for col in range(len(self.starMap[row])):
                if self.starMap[row][col] != None: 
                    withinCellBounds = abs(x - self.starMap[row][col].x) < self.resolution/2.0
                    withinCellBounds = withinCellBounds and abs(y - self.starMap[row][col].y) < self.resolution/2.0
                    if withinCellBounds:
                        return self.starMap[row][col]
        print("A* Server: No grid cell found containing given point")
        return None  

    # estimates distance between two nodes
    def heuristic(self, currentNode, goalNode):        
        diffX = abs(currentNode.x - goalNode.x)
        diffY = abs(currentNode.y - goalNode.y)        
        distance = math.hypot(diffX, diffY)                
        costFactor = .1 + 9*currentNode.heuristicCost/1000.0
        if currentNode.isWall: 
            costFactor = 999999
        return distance*costFactor

    # gets the node with the smallest known cost, assuming self.frontierNodes is not empty
    def getMinimumNode(self, startIsWall):        
        minNode = self.frontierNodes[0]
        if startIsWall:
            return minNode
        minimum = minNode.predictedCost
        for node in self.frontierNodes:
            if node.predictedCost < minimum:
                minimum = node.predictedCost
                minNode = node 
        return minNode

    # gets this node's neighbors in the A* map if they exist
    def getNeighbors(self, node):
        row = node.row
        col = node.col

        neighbors = []
        isNotTopRow = row-1 >= 0
        isNotBottomRow = row+1 < len(self.starMap)
        isNotLeftColumn = col-1 >= 0
        isNotRightColumn = col+1 < len(self.starMap[row])
        # make 4-connected
        if isNotTopRow: 
            neighbors.append(self.starMap[row-1][col])        
        if isNotBottomRow:
            neighbors.append(self.starMap[row+1][col])
        if isNotLeftColumn:
            neighbors.append(self.starMap[row][col-1])
        if isNotRightColumn:
            neighbors.append(self.starMap[row][col+1])   
        # make 8-connected
        if isNotTopRow and isNotLeftColumn: 
            neighbors.append(self.starMap[row-1][col-1])  
        if isNotTopRow and isNotRightColumn: 
            neighbors.append(self.starMap[row-1][col+1])  
        if isNotBottomRow and isNotLeftColumn:
            neighbors.append(self.starMap[row+1][col-1])  
        if isNotBottomRow and isNotRightColumn: 
            neighbors.append(self.starMap[row+1][col+1])   
        return [node for node in neighbors if node != None] # remove uninitialized nodes

    # create array of nodes in path in reverse from goal to start
    def buildPath(self, previousStepTo, endNode):
        path = []
        while endNode in previousStepTo: 
            path.append(endNode)
            endNode = previousStepTo[endNode] # step back making the path one node shorter and closer to the start
        path.append(endNode) # ensure the start node is included on the path
        return path

    # creates orientation and position directions to follow to execute the given path
    def calculateWaypoints(self, path): 
        waypoints = Path()
        waypoints.header.seq = 1
        waypoints.header.stamp = rospy.Time.now()
        waypoints.header.frame_id = self.frameID

        if path != []:
            prevNode = path[0]
            delta = (0, 0) # final orientation is facing east       
            for currentNode in path[1:]: # will not throw index error if path only one element        
                newDelta = ((prevNode.x-currentNode.x)*1000000//1, (prevNode.y-currentNode.y)*1000000//1)                        
                if (delta != newDelta):                
                    waypoints.poses.insert(0, self.createPoseStamped(prevNode.x, prevNode.y, math.atan2(delta[1], delta[0])))
                delta = newDelta
                prevNode = currentNode 
            waypoints.poses.insert(0, self.createPoseStamped(prevNode.x, prevNode.y, math.atan2(delta[1], delta[0])))
            print("A* Server: Got a path, here ya go!")
        return waypoints

    # create PoseStamped message given x, y, and radian orientation
    def createPoseStamped(self, x, y, radians):   
        pose = PoseStamped()
        pose.header.seq = 1
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = self.frameID
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0
        quaternionArray = tf.transformations.quaternion_about_axis(radians, Z_AXIS)
        pose.pose.orientation.x = quaternionArray[0]         
        pose.pose.orientation.y = quaternionArray[1]
        pose.pose.orientation.z = quaternionArray[2]
        pose.pose.orientation.w = quaternionArray[3]
        return pose

     # convert array of StarNodes to grid cells
    def createGridCellsFromArray(self, nodeArray):        
        grid = GridCells()
        grid.header.seq = 1
        grid.header.stamp = rospy.Time.now()
        grid.header.frame_id = self.frameID
        grid.cell_width = self.resolution
        grid.cell_height = self.resolution
        for node in nodeArray:   
            point = Point(node.x, node.y, 0)
            grid.cells.append(point) 
        return grid

# The program's primary executing section
if __name__ == '__main__':
    AStarServiceServer()
