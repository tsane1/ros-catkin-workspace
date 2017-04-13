#!/usr/bin/env python

"""
This file contains all code for RBE 3002 Lab 3 at WPI

Authors:
	- Tanuj Sane
  - Matthew Lamare

Since:
	- 4/5/2017
	- 4/3/2017
	- 3/30/2017

Version:
	- 1.2 Refactoring to run as a separate node providing a service
	- 1.1 Moved A* implementation into `astar.py`
	- 1.0 Initial Commit
"""

import rospy
from std_msgs.msg import Float64
from nav_msgs.msg import MapMetaData, OccupancyGrid
from nav_msgs.msg import GridCells, Path
from geometry_msgs.msg import PoseStamped, Point
from random import randint
from astar import Node, a_star as astar, a_star_small as astar_srv
from math import atan2
from nav_msgs.srv import GetPlan
"""
Callback function for subscription to the map

Parameters:
	- msg	{MapMetaData} The incoming message from the subscription
"""
def metadata_callback(msg):
	global mapSize
	mapSize = []
	mapSize.append(msg.width)
	mapSize.append(msg.height)

"""
Callback function for subscription to the map

Parameters:
	- msg	{OccupancyGrid} The incoming message from the subscription
"""
def map_callback(msg):
	global mapMsg
	mapMsg = msg

################################################################################
# Coloring
################################################################################
def makePoint():
	#randomizer
	randWidth = randint(0, 2)
	randHeight = randint(0, 2)

	#create a point
	point = Point()
	point.x = randWidth
	point.y = randHeight
	
	#print point
	return point


def coloring():
	gridCells = GridCells()

	gridCells.header.stamp = rospy.Time()
	gridCells.header.frame_id = 'map'

	#set size of points
	gridCells.cell_width = 0.25
	gridCells.cell_height = 0.25

	#Makes a random number (0 to 100) in random locaions within the grid
	for i in range(0,10):
		gridCells.cells.append(makePoint())
	
	global point_pub
	point_pub.publish(gridCells)
	print "done"
	rospy.sleep(rospy.Duration(0.5))
################################################################################

"""
The path planner subroutine

Parameters:
	- request	{Dict} The incoming request

Returns:
	- A {Path} message containing the path from start to goal
"""
def get_path(request):
	print "Recieved a request for a plan!"
	global mapMsg

	start = request.start
	goal = request.goal

	try: mapMsg
	except NameError: print 'NO MAP FOUND'
	else:
		# Initialize map as a double array of Node objects
		theMap = [[0 for x in range(mapMsg.info.width)] for y in range(mapMsg.info.height)]

		# Populate double array from row-oriented single array
		row = col = -1
		for i in range(len(mapMsg.data)):
			if i % mapMsg.info.width == 0:
				row += 1
				col = 0
			theMap[row][col] = Node(row, col, goal)
			col += 1
		
			path_msg = Path()
			path_msg.header.stamp = rospy.Time()
			path_msg.header.frame_id = 'map'

			lastx = 0
			lasty = -1 

			# Do A*, then convert output path into Poses and append to Path
			for point in astar_srv(theMap, start, goal):
				poseStamped = PoseStamped()
				x = point.x
				y = point.y

				poseStamped.header.stamp = rospy.Time()

				poseStamped.pose.position.x = x
				poseStamped.pose.position.y = y
				poseStamped.pose.orientation.x = x
				poseStamped.pose.orientation.y = y
				poseStamped.pose.orientation.w = atan2(y - lasty, x - lastx)

				lastx = x
				lasty = y

				path_msg.poses.append(poseStamped)

			print "Got a path, here ya go..."
			return path_msg
	

# Main
if __name__ == '__main__':
	global mapMsg

	start = PoseStamped()
	start.pose.position.x = 0
	start.pose.position.y = 0

	goal = PoseStamped()
	goal.pose.position.x = 5
	goal.pose.position.y = 2

	rospy.init_node('tsane_mmlamare_lab3')

	# Publishers
	point_pub = rospy.Publisher('/a_star/frontier', GridCells, queue_size=10)
	path_pub = rospy.Publisher('/a_star/path', Path, queue_size=10)

	# Subscribers
	metadata_sub = rospy.Subscriber('/map_metadata', MapMetaData, metadata_callback)
	map_sub = rospy.Subscriber('/map', OccupancyGrid, map_callback)

	print "Starting Lab 3"

	# planner = rospy.Service('path_planner', GetPlan, get_path)
	# planner.spin()

	while raw_input() != 'q':
		try:
			print start
			print goal
			mapMsg
		except NameError:
			print 'NO DATA FOUND'
		else:
			theMap = [[0 for x in range(mapMsg.info.width)] for y in range(mapMsg.info.height)]

			row = col = -1
			for i in range(len(mapMsg.data)):
				if i % mapMsg.info.width == 0:
					row += 1
					col = 0
				theMap[row][col] = Node(row, col, goal)
				col += 1
		
			path_msg = Path()
			path_msg.header.frame_id = 'map'
			lastx = 0
			lasty = -1 #have its inital pose pointing up the y-axis
			for point in astar(theMap, start, goal):
				pose = PoseStamped()
				x = point.x
				y = point.y

				pose.header.stamp = rospy.Time()
				pose.header.frame_id = 'map'

				pose.pose.position.x = x
				pose.pose.position.y = y
				pose.pose.orientation.x = x
				pose.pose.orientation.y = y
				pose.pose.orientation.w = atan2(y - lasty,x - lastx)

				lastx = x
				lasty = y

				path_msg.poses.append(pose)

			path_pub.publish(path_msg)
			rospy.sleep(rospy.Duration(0.5))
	
	print "Lab 3 Finished"
	
