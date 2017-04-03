"""
File: astar.py

An implementation of the A* algorithm for ROS

Authors:
	- Tanuj Sane
	- Matthew Lamare

Since:
	- 4/4/2017

Version:
	- 1.0 Initial commit

"""

# Imports
from heapq import *
import math


"""
A class containing Node information
"""
class Node:
	def __init__(self, x, y, goal):
		self.x = x
		self.y = y


		self.g = 0
		self.h = math.sqrt((self.x - goal.pose.position.x) ** 2 + (self.y - goal.pose.position.y) ** 2)

	def update_g(self, parent):
		self.g = parent.g + 1

	def __lt__(self, other):
		return (self.g + self.h) <= (other.g + other.h)
		

def printNode(node):
	print "(" + str(node.x) + ", " + str(node.y) + ")"

def reached(goal, current):
	return goal.pose.position.x == current.x and goal.pose.position.y == current.y

"""
An implementation of the A* path planning algorithm

Parameters:
	- theMap {arr[][]} The 
"""
def a_star(theMap, start, goal):	
	"""
	SHIT TO DO: coloring based on open vs closed vs unexplored, path coloring, waypoints
	"""
	
	gridCells = GridCells()
	gridCells = GridCells()

	gridCells.header.stamp = rospy.Time()
	gridCells.header.frame_id = 'map'

	#set size of points
	gridCells.cell_width = 0.25
	gridCells.cell_height = 0.25

	point_pub = rospy.Publisher('/a_star/frontier', GridCells, queue_size=10)

	closedSet = []
	openSet = [
		theMap[start.pose.position.x][start.pose.position.y]
	]

	while not len(openSet) == 0:
		heapify(openSet)
		current = openSet[0]
	
		if reached(goal, current):
			#return reconstruct_path(current, current.parent) #still need to write this
		
		openSet.remove(current)
		closedSet.append(current)
  
		for n in getNeighbors(current, goal):
			print "Using "
			printNode(current)
			print "as current"

			if n in closedSet:
				continue
			printNode(n)
			tentative_gCost = current.g + 1

			n.g = 100
			if n not in openSet:
				openSet.append(n)			
			elif tentative_gCost >= n.g:
				continue

			n.parent = current
			n.g = tentative_gCost
			f = n.g + n.h
			print f
			closedSet.append(n)

			#while raw_input() != 'c': continue

	print "A* Fucked Up"
	exit()

"""
This is a shitty subroutine that gets all the neighbors for a node
"""
def getNeighbors(currentNode, goal):
	returnList = []
	x = currentNode.x
	y = currentNode.y
	addNode(returnList, x -1, y - 1, goal)
	addNode(returnList, x, y -1, goal)
	addNode(returnList, x + 1, y - 1, goal)
	addNode(returnList, x - 1, y, goal)
	addNode(returnList, x + 1, y, goal)
	addNode(returnList, x - 1, y + 1, goal)
	addNode(returnList, x, y + 1, goal)
	addNode(returnList, x + 1, y + 1, goal)

	return returnList

"""
This is an even shittyer subsubroutine for above that adds nodes

rev 2.0: now includes error handling
"""
def addNode(nodeList, x, y, goal):
	if x < 0 or y < 0: 
		return
	newNode = Node(x,y, goal)
	nodeList.append(newNode)

