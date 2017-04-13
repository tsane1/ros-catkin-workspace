"""
Map Checker
This file contains the map checking function to determine if it is fully explored

Author: 
	- Matthew Lamare

Reversion:
	- 1.0, inital version, 4/13/17
"""


"""
This Function determines if a Map has been full explored

params:
	- theMap: The map to be tested

output:
	- True: The map is complete
	- False: The map is not complete
"""
def isMapComplete(theMap):
	for row in theMap:
		for node in row:
			if isFrontier(node):
				return False
	return True

"""
This Function determines if a node is an unexplored frontier edge

params:
	- testNode: The node to be tested if it is a frontier

output:
	- True: testNode is a frontier
	- False: testNode is not a frontier
"""
def isFrontier(testNode):
	return testNode.isWall != isWall or testNode.knownCost == -1
