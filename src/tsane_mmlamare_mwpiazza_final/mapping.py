"""
[DESCRIPTION]

Authors:
	- Tanuj Sane
	- Matthew Lamare
	- Matthew Piazza

Since:
	- 4/13/2017

Version:
	- 1.0 Initial commit
"""

import rospy
from callbacks import AsyncValue

"""
Encapsulation of the map and all necessary functions
"""
class TheMap:
	def __init__(self):
		self.theMap = []

	def update(self, msg):
		row = col = -1
		for i in range(len(msg.data))
			if i % msg.info.width == 0:
				row += 1
				col = 0

			self.the_map[row][col] = 
			col += 1

	def frontier(self):
		


# Main
if __name__ == '__main__':
	"""
	1. Spin in place to generate partial map
	2. Find the frontier
	3. Find centroid of frontier clusters
	4. Determine heuristic for frontier clusters
	5. A* over to that centroid point
	6. Rinse and repeat
	"""	
	# MAP!
	the_map = TheMap()

	# Publishers
	

	# Subscribers
	sub_map = rospy.Subscriber("/map", OccupancyGrid, the_map.update, queue_size=1)
	