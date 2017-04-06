#!/usr/bin/env python

"""
Dedicated path planning node implementing A*, providing a path planning service

Authors:
	- Tanuj Sane
	- Matthew Lamare
	- Matthew Piazza

Since:
	- 4/6/2017

Version:
	- 1.0 Initial commit
"""

# Imports
import rospy
import math
from callbacks import AsyncValue
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import MapMetaData, OccupancyGrid
from nav_msgs.srv import GetPlan
import heapq

distance = lambda x1,y1,x2,y2: math.sqrt((x2 - x1) ** 2 + (y1 - y2)**2)

"""
Encapsulation of A* node data
"""
class Node:
	def __init__(self, x, y, p, h):
		self.x = x
		self.y = y
	
		self.p = p	

		self.g = 0
		self.h = h

	def __lt__(self, other):
		return (self.g + self.h) <= (other.g + other.h)

	def __eq__(self, other):
		return self.x == other.x and self.y == other.y
		
"""
The A* planning algorithm

Parameters:
	- theMap	{[][]}				A 2D array describing the occupancy grid of the current space
	- start		{PoseStamped}	The starting pose
	- goal		{PoseStamped}	The goal location
"""
def run(theMap, start, goal):	
	# The set of nodes that has been seen already
	closedSet = []

	# The frontier
	openSet = [
		theMap[int(start.pose.position.x)][int(start.pose.position.y)]
	]
	
	# A flag to make sure we produce the correct print statement
	success = False 

	while not len(openSet) == 0:
		heapify(openSet)
		current = openSet[0]
	
		if reached(goal, current):
			closedSet.append(current)
			return closedSet
	
		# Mark current Node as closed
		openSet.remove(current)
		closedSet.append(current)
  
		for n in get_neighbors(current, goal):
			if n in closedSet:
				continue

			tentative_gCost = current.g + distance(current.x, current.y, n.x, n.y)

			if n not in openSet:
				if not n.p == 100: openSet.append(n)			
				
			elif tentative_gCost >= n.g:
				continue

			n.g = tentative_gCost
		
	if not success:
		print "A* Fucked Up"

"""
Consolidates neighbors of current node

Parameters:
	- theMap	{[][]} The map to use to find neighbors
	- current {Node} The node whose neighbors to find

Return:
	- A list of the neighbors of the current node
"""
def get_neighbors(theMap, currentNode):
	global asyncMapMetaData
	w, h = asyncMapMetaData.unwrap()

	# Aliases for easy typing	
	x = currentNode.x
	y = currentNode.y

	out = []
	for i in range(x - 1, x + 1):
		for j in range(y - 1, y + 1):
			if i < 0 or j < 0 or i > w or j > h:
				continue		
			else: out.append(theMap[i][j])

	return out

"""

"""
def create_posestamped(self, x, y, radians):   
	pose = PoseStamped()
	
	pose.header.seq = 1
	pose.header.stamp = rospy.Time.now()
	pose.header.frame_id = 'map'
	
	pose.pose.position.x = x
	pose.pose.position.y = y
	pose.pose.position.z = 0

	quaternionArray = tf.transformations.quaternion_about_axis(radians, (0, 0, 1))
	pose.pose.orientation.x = quaternionArray[0]         
	pose.pose.orientation.y = quaternionArray[1]
	pose.pose.orientation.z = quaternionArray[2]
	pose.pose.orientation.w = quaternionArray[3]
  
	return pose

"""
The path planner subroutine

Parameters:
	- request	{Dict} The incoming request

Returns:
	- A {Path} message containing the path from start to goal
"""
def get_path(request):
	global asyncOccupancyGrid
	global asyncMapMetaData	

	w, h = asyncMapMetaData.unwrap()
	og = asyncOccupancyGrid.unwrap() 

	start = request.start
	goal = request.goal

	# Initialize map as a double array of Node objects
	theMap = [[0 for x in range(w)] for y in range(h)]

	# Populate double array from row-oriented single array
	row = col = p = -1
	for i in range(len(og.data)):
		p = og.data[i]
		if i % w == 0:
			row += 1
			col = 0

		h = distance(row, col, goal.position.pose.x, goal.position.pose.y)
	
		theMap[row][col] = Node(row, col, p, h)
		col += 1
	
		path_msg = Path()
		path_msg.header.stamp = rospy.Time()
		path_msg.header.frame_id = 'map'
		
		lastx = 0
		lasty = -1 

		# Do A*, then convert output path into Poses and append to Path
		for point in run(theMap, start, goal):
			poseStamped = PoseStamped()

			# Alias for easy typing			
			x = point.x
			y = point.y

			dx = lastx - x
			dy = lasty -y

			path_msg.poses.append(create_posestamped(lastx, lasty, math.atan2(dy, dx)))
			
			# Update previous values
			lastx = x
			lasty = y

		print "Got a path, here ya go..."
		return path_msg

"""
Callback for /map_metadata subscription

Parameters:
	- msg {MapMetaData} The metadata for the map
"""
def mapmetadata_callback(msg):
	global asyncMapMetaData
	asyncMapMetaData = AsyncValue()
	asyncMapMetaData.wrap((msg.width, msg.height))
	asyncMapMetaData.unlock()

"""
Callback for /map subscription

Parameters:
	- msg {OccupancyGrid} The metadata for the map
"""
def map_callback(msg):
	global asyncOccupancyGrid
	asyncOccupancyGrid = AsyncValue()
	asyncOccupancyGrid.wrap(msg)
	asyncOccupancyGrid.unlock()
	
if __name__ == '__main__':
	rospy.init_node('path_planner')
	
	# Subscribers
	metadata_sub = rospy.Subscriber('/map_metadata', MapMetaData, mapmetadata_callback)
	map_sub = rospy.Subscriber('/map', OccupancyGrid, map_callback)

	global asyncOccupancyGrid
	global asyncMapMetaData
	
	asyncOccupancyGrid = AsyncValue()
	asyncMapMetaData = AsyncValue()

	# Spin until callback values are populated
	print 'Waiting for map data...'
	while asyncOccupancyGrid.is_locked() and asyncMapMetaData.is_locked():
		continue
	
	print 'Map data recieved! Starting service...'
	# Start service
	planner = rospy.Service('path_planner', GetPlan, get_path)
	planner.spin()
	