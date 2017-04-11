#!/usr/bin/env python

""" 
ROS Server calculating the path between two points on a map using the A* algorithm

Authors: Matthew Piazza
Date: 4/10/17
"""

import math
import rospy, tf
from nav_msgs.msg import GridCells, Path
from geometry_msgs.msg import Point, PoseStamped
from tsane_mmlamare_mwpiazza_lab4.srv import *

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
class AStarServiceServer():
    # set up server  
    def __init__(self):        	    
        rospy.init_node('a_star_server_apo')
        s = rospy.Service('a_star', AStar, self.handleAStar)
        rospy.spin()                    

    # run A* algorithm and return fields
    def handleAStar(self, msg):
        self.readOccupancyGridMap(msg.map)        
        self.frameID = msg.frameID.data      
        waypointArray = self.aStar(msg.start.position.x, msg.start.position.y, msg.goal.position.x, msg.goal.position.y)
        waypoints = self.calculateWaypoints(waypointArray)        
        return AStarResponse(waypoints)

    # convert occupancy grid from RViz to 2D matrix map of StarNodes
    def readOccupancyGridMap(self, map):        
        mapRows = map.info.height
        mapColumns = map.info.width
        self.resolution = map.info.resolution        
        self.origin = map.info.origin
        
        # create map of nodes
        grid = [[map.data[(row*mapColumns) + col] for col in range(mapColumns)] for row in range(mapRows)]  
        self.starMap = [[None for col in range(len(grid[row]))] for row in range(len(grid))]  
        for row in range(len(grid)):
            for col in range(len(grid[row])):
                if grid[row][col] != -1:
                    x = (col*self.resolution) + self.origin.position.x + self.resolution/2.0
                    y = (row*self.resolution) + self.origin.position.y + self.resolution/2.0
                    isWall = grid[row][col] == 100                
                    self.starMap[row][col] = StarNode(x, y, isWall, row, col)        

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
	            currentNode = self.getMinimumNode()
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
	                if neighborNode not in self.frontierNodes and not neighborNode.isWall:
	                    self.frontierNodes.append(neighborNode)
	                elif neighborKnownCost >= neighborNode.knownCost and neighborNode.knownCost != -1:
	                    continue

	                # update neighbor node with new optimal costs
	                previousStepTo[neighborNode] = currentNode
	                neighborNode.knownCost = neighborKnownCost
	                neighborNode.predictedCost = neighborNode.knownCost + self.heuristic(neighborNode, goalNode)

        print("A* could not find a path :( ")
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
        print("No grid cell found containing given point")
        return None  

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

        if path != []:
        	prevNode = path[0]
        	delta = (0, 0) # final orientation is facing east   	
        	for currentNode in path[1:]: # will not throw index error if path only one element    	
        		newDelta = ((prevNode.x-currentNode.x)*1000000//1, (prevNode.y-currentNode.y)*1000000//1)    		    		
        		if (delta != newDelta):    			
        			waypoints.poses.append(self.createPoseStamped(prevNode.x, prevNode.y, math.atan2(delta[1], delta[0])))
        		delta = newDelta
        		prevNode = currentNode 
        	waypoints.poses.append(self.createPoseStamped(prevNode.x, prevNode.y, math.atan2(delta[1], delta[0])))
        	print("Got a path, here ya go!")
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