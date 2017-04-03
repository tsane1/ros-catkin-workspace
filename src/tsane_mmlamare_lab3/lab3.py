#!/usr/bin/env python

"""
This file contains all code for RBE 3002 Lab 3 at WPI

Authors:
	- Tanuj Sane
  - Matthew Lamare

Since:
	- 3/30/2017

Version:
	- 1.0 Initial Commit
"""

import rospy
from std_msgs.msg import Float64
from nav_msgs.msg import MapMetaData, OccupancyGrid
from nav_msgs.msg import GridCells
from geometry_msgs.msg import PoseStamped, Point
from random import randint
import heapq

class Node:
	def __init__(self, x, y):
		self.x = x
		self.y = y


		self.g = 100
		global goal
		self.h = math.sqrt((self.x - goal.x) ** 2 + (self.y - goal.y) ** 2)

	def update_g(self, parent):
		self.g = parent.g + 1
		
		

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
	theMap = [[0 for x in range(msg.info.width)] for y in range(msg.info.height)]

	row = col = 0
	for i in range(len(msg.data)):
		if i % msg.info.width == 0:
			row += 1
			col = 0
		theMap[row][col] = Node(row, col)
		

	

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

"""
An implementation of the A* path planning algorithm
"""
def a_star():
	global theMap, start, goal
	
	closedSet = []
	openSet = [
		theMap[start.x][start.y]
	]

	while not openSet:
		openSet = openSet.heapify()
		current = openSet[0]
	
		if current == goal:
			return "IT WORKED"
			#return reconstruct_path(current, current.parent) #still need to write this
		
		openSet.remove(current)
		closedSet.append(current)
  
		for n in getNeighbors(current):
			if n in closedSet:
				continue
		
			tentative_gCost = current.g + 1

			if n not in openset:
				openSet.append(n)
	
			elif tentative_gCost >= n.g:
				continue

			n.parent = current
			n.g = tentative_gCost
			f = n.g + n.h

	return "A* Fucked Up"


def makePoint():
	#randomizer
	randWidth = randint(0, 10)
	randHeight = randint(0, 10)

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
	gridCells.cell_width = 5
	gridCells.cell_height = 5

	#Makes a random number (0 to 100) in random locaions within the grid
	for i in range(0,10):
		gridCells.cells.append(makePoint())
		
	global point_pub
	point_pub.publish(gridCells)

# Main
if __name__ == '__main__':
	rospy.init_node('tsane_mmlamare_lab3')

	# Publishers
	global point_pub
	point_pub = rospy.Publisher('/a_star/frontier', GridCells, queue_size=10)
	
	# Subscribers
	metadata_sub = rospy.Subscriber('/map_metadata', MapMetaData, metadata_callback)
	map_sub = rospy.Subscriber('/map', OccupancyGrid, map_callback)
	entropy_sub = rospy.Subscriber('/slam_gmapping/entropy', Float64, entropy_callback)

	start_sub = rospy.Subscriber('/initalpose', PoseStamped, start_callback)
	goal_sub = rospy.Subscriber('/move_base_simple/goal', PoseStamped, goal_callback)

	print "Starting Lab 3"

	while raw_input() != 'q': 
		a_star()
	#while not rospy.is_shutdown(): 
	#	rospy.spin()

	print "Lab 3 Finished"
	
