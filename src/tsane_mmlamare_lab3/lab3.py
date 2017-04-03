#!/usr/bin/env python

"""
This file contains all code for RBE 3002 Lab 3 at WPI

Authors:
	- Tanuj Sane
  - Matthew Lamare

Since:
	- 4/3/2017
	- 3/30/2017

Version:
	- 1.1 Moved A* stuff into `astar.py`
	- 1.0 Initial Commit
"""

import rospy
from std_msgs.msg import Float64
from nav_msgs.msg import MapMetaData, OccupancyGrid
from nav_msgs.msg import GridCells
from geometry_msgs.msg import PoseStamped, Point
from random import randint

from astar import Node, a_star as astar
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
	global theMap
	global goal

	theMap = [[0 for x in range(msg.info.width)] for y in range(msg.info.height)]

	row = col = -1
	for i in range(len(msg.data)):
		if i % msg.info.width == 0:
			row += 1
			col = 0
		theMap[row][col] = Node(row, col, goal)
		col += 1
		
"""
Callback function for subscription to the map

Parameters:
	- msg	{Float64} The incoming message from the subscription
"""
def entropy_callback(msg):
	pass

"""
Callback function for subscription to the starting pose

Parameters:
	- msg	{PoseStamped} The incoming message from the subscription
"""
def start_callback(msg):
	global start
	start = msg.pose

"""
Callback function for subscription to the map

Parameters:
	- msg	{PoseStamped} The incoming message from the subscription
"""
def goal_callback(msg):
	global goal
	goal = msg.pose


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

# Main
if __name__ == '__main__':
	global start
	start = PoseStamped()
	start.pose.position.x = 0
	start.pose.position.y = 0

	global goal
	goal = PoseStamped()
	goal.pose.position.x = 5
	goal.pose.position.y = 5
	
	rospy.init_node('tsane_mmlamare_lab3')

	# Publishers
	
	# Subscribers
	metadata_sub = rospy.Subscriber('/map_metadata', MapMetaData, metadata_callback)
	map_sub = rospy.Subscriber('/map', OccupancyGrid, map_callback)
	entropy_sub = rospy.Subscriber('/slam_gmapping/entropy', Float64, entropy_callback)

	start_sub = rospy.Subscriber('/initalpose', PoseStamped, start_callback)
	goal_sub = rospy.Subscriber('/move_base_simple/goal', PoseStamped, goal_callback)

	print "Starting Lab 3"

	while raw_input() != 'q': 
		coloring()

	print "Lab 3 Finished"
	
