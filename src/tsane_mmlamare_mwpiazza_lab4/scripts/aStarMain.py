#!/usr/bin/env python

""" 
ROS Node calculating the path between two points on a map using the A* algorithm

Authors: Joseph Lombardi, Matthew Piazza
Date: 4/5/17
"""

import math
import rospy, tf
from nav_msgs.msg import GridCells, OccupancyGrid, Path
from geometry_msgs.msg import Point, PoseWithCovarianceStamped, PointStamped, PoseStamped

PI = 3.141592653589
Z_AXIS = (0, 0, 1)

# A* Map Point 
class StarNode():
    def __init__(self, x, y, isWall, row, col):
        self.x = x
        self.y = y            
        self.isWall = isWall
        self.row = row
        self.col = col    
        self.knownCost = -1
        self.predictedCost = -1

# ROS node 
class Lab3Node():
    def __init__(self):  
        # set up topics      
        self.subMap = rospy.Subscriber("/map", OccupancyGrid, self.readOccupancyGridMap, queue_size=1) # Callback function to load map on initial load        
        self.pubFrontier = rospy.Publisher('/frontier', GridCells, queue_size=10) 
        self.pubVisited = rospy.Publisher('/visited', GridCells, queue_size=10) 
        self.pubPath = rospy.Publisher('/path', GridCells, queue_size=10)         
        self.pubWaypoints = rospy.Publisher('/waypoints', Path, queue_size=10)         

        # set up control parameters
        self.hz = 10        
        self.spinRate = rospy.Rate(self.hz)        
        self.frameID = "map"                     
        
        rospy.sleep(rospy.Duration(1, 0)) # wait for a moment
        self.run()

    # run continuously, act upon subscriber callbacks
    def run(self):
        self.running = True
        while self.running:
            self.spinRate.sleep()

    # convert occupancy grid from RViz to 2D matrix map of StarNodes
    def readOccupancyGridMap(self, map):        
        mapRows = map.info.height
        mapColumns = map.info.width
        self.resolution = map.info.resolution        
        self.origin = map.info.origin
        grid = [[map.data[(row*mapColumns) + col] for col in range(mapColumns)] for row in range(mapRows)]  
        self.createMap(grid) 
        self.setupPointMessaging()      

    # creates map of A* nodes given an occupancy grid
    def createMap(self, occupancyMatrix):  
        self.starMap = [[None for col in range(len(occupancyMatrix[row]))] for row in range(len(occupancyMatrix))]  
        for row in range(len(occupancyMatrix)):
            for col in range(len(occupancyMatrix[row])):
                if occupancyMatrix[row][col] != -1:
                    x = (col*self.resolution) + self.origin.position.x + self.resolution/2.0
                    y = (row*self.resolution) + self.origin.position.y + self.resolution/2.0
                    isWall = occupancyMatrix[row][col] == 100                
                    self.starMap[row][col] = StarNode(x, y, isWall, row, col)        

    # once the map is created, this subscribes to point topics and set their control variables
    def setupPointMessaging(self):
        self.startIsSet = False
        self.endIsSet = False
        self.subStart = rospy.Subscriber("/initialpose", PoseWithCovarianceStamped, self.setStart, queue_size=1) # Callback function to load map on initial load
        self.pubStart = rospy.Publisher('/startPoint', PointStamped, queue_size=10) 
        self.subEnd = rospy.Subscriber("/clicked_point", PointStamped, self.setEnd, queue_size=1) # Callback function to load map on initial load
        self.pubEnd = rospy.Publisher('/endPoint', PointStamped, queue_size=10)               

    # sets start of A* based on 2D Pose Estimate
    def setStart(self, poseMsg):
        self.startNode = self.getNodeFromXY(poseMsg.pose.pose.position.x, poseMsg.pose.pose.position.y)        
        self.pubStart.publish(self.createPointStamped(self.startNode.x, self.startNode.y))
        self.startIsSet = True
        if self.startIsSet and self.endIsSet:
            self.planAndPublishPath()
        else:
            print("Please set end point with RViz Publish Point tool")       

    # sets end of A* based on Publish Point
    def setEnd(self, pointStampedMsg):
        self.goalNode = self.getNodeFromXY(pointStampedMsg.point.x, pointStampedMsg.point.y)        
        self.pubEnd.publish(self.createPointStamped(self.goalNode.x, self.goalNode.y))
        self.endIsSet = True
        if self.startIsSet and self.endIsSet:
            self.planAndPublishPath() 
        else:
            print("Please set start point with RViz 2D Pose Estimate tool")       

    # finds center of grid cell containing coordinate
    def getNodeFromXY(self, x, y):
        for row in range(len(self.starMap)):
            for col in range(len(self.starMap[row])):
                if self.starMap[row][col] != None: 
                    withinCellBounds = abs(x - self.starMap[row][col].x) < self.resolution/2.0
                    withinCellBounds = withinCellBounds and abs(y - self.starMap[row][col].y) < self.resolution/2.0
                    if withinCellBounds:
                        return self.starMap[row][col]
        print("No grid cell found containing given point")
        return None

    # run A* algorithm and send grid cells to RViz
    def planAndPublishPath(self):
        path = self.aStar()
        if path != []:
            self.calculateWaypoints(path)
            self.pubPath.publish(self.createGridCellsFromArray(path))
            self.pubVisited.publish(self.createGridCellsFromArray(self.visitedNodes))
            self.pubFrontier.publish(self.createGridCellsFromArray(self.frontierNodes))
            print("A* Finished!")

    # calculates best path between the set start and end nodes
    def aStar(self):
        self.visitedNodes = [] # list of evaluated nodes
        self.frontierNodes = [self.startNode] # list of nodes needing to be evaluated

        previousStepTo = {} # linked list of most efficient previous steps
        self.startNode.knownCost = 0
        self.startNode.predictedCost = self.heuristic(self.startNode, self.goalNode)

        while self.frontierNodes != []:
            # check for finished
            currentNode = self.getMinimumNode()
            if currentNode == self.goalNode:        
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
                if neighborNode not in self.frontierNodes and not neighborNode.isWall:
                    self.frontierNodes.append(neighborNode)
                elif neighborKnownCost >= neighborNode.knownCost and neighborNode.knownCost != -1:
                    continue

                # update neighbor node with new optimal costs
                previousStepTo[neighborNode] = currentNode
                neighborNode.knownCost = neighborKnownCost
                neighborNode.predictedCost = neighborNode.knownCost + self.heuristic(neighborNode, self.goalNode)

        print("A* could not find a path :( ")
        return []

    # estimates distance between two nodes
    def heuristic(self, current, goalNode):        
        diffX = abs(current.x - goalNode.x)
        diffY = abs(current.y - goalNode.y)                
        return math.hypot(diffX, diffY)        

    # gets the node with the smallest known cost, assuming self.frontierNodes is not empty
    def getMinimumNode(self):        
        minNode = self.frontierNodes[0]
        minimum = minNode.predictedCost
        for node in self.frontierNodes:
            if node.predictedCost < minimum and not node.isWall:
                minimum = node.predictedCost
                minNode = node 
        return minNode

    # gets this node's neighbors in the A* map if they exist
    def getNeighbors(self, currentNode):
        neighbors = []
        isNotTopRow = currentNode.row-1 >= 0
        isNotBottomRow = currentNode.row+1 < len(self.starMap)
        isNotLeftColumn = currentNode.col-1 >= 0
        isNotRightColumn = currentNode.col+1 < len(self.starMap[currentNode.row])
        # make 4-connected
        if isNotTopRow: 
            neighbors.append(self.starMap[currentNode.row-1][currentNode.col])        
        if isNotBottomRow:
            neighbors.append(self.starMap[currentNode.row+1][currentNode.col])
        if isNotLeftColumn:
            neighbors.append(self.starMap[currentNode.row][currentNode.col-1])
        if isNotRightColumn:
            neighbors.append(self.starMap[currentNode.row][currentNode.col+1])   
        # make 8-connected
        if isNotTopRow and isNotLeftColumn: 
            neighbors.append(self.starMap[currentNode.row-1][currentNode.col-1])  
        if isNotTopRow and isNotRightColumn: 
            neighbors.append(self.starMap[currentNode.row-1][currentNode.col+1])  
        if isNotBottomRow and isNotLeftColumn:
            neighbors.append(self.starMap[currentNode.row+1][currentNode.col-1])  
        if isNotBottomRow and isNotRightColumn: 
            neighbors.append(self.starMap[currentNode.row+1][currentNode.col+1])   
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

        prevNode = path[0]
        delta = (0, 0) # final orientation is facing east       
        for currentNode in path[1:]: # will not throw index error if path only one element        
            newDelta = ((prevNode.x-currentNode.x)*1000000//1, (prevNode.y-currentNode.y)*1000000//1)                        
            if (delta != newDelta):                
                waypoints.poses.append(self.createPoseStamped(prevNode.x, prevNode.y, math.atan2(delta[1], delta[0])))
            delta = newDelta
            prevNode = currentNode 
        waypoints.poses.append(self.createPoseStamped(prevNode.x, prevNode.y, math.atan2(delta[1], delta[0])))
        self.pubWaypoints.publish(waypoints)

    # create PointStamped message given x and y
    def createPointStamped(self, x, y):   
        point = PointStamped()
        point.header.seq = 1
        point.header.stamp = rospy.Time.now()
        point.header.frame_id = self.frameID
        point.point.x = x
        point.point.y = y        
        return point

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
    # Change this node name to include your username
    rospy.init_node('mpiazza_jlompardi_lab3')
    node = Lab3Node()