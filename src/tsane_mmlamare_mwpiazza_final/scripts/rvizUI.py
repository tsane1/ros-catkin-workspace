#!/usr/bin/env python

""" 
ROS Server calculating the path between two points on a map using the A* algorithm

Author:
    - Tanuj Sane
    - Matthew Lamare
    - Matthew Piazza

Updated: 4/26/17
"""

# Imports
import rospy, tf
from std_msgs.msg import String
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped, PointStamped
import padding

# Constants
PADDING = .4 # meters

"""
ROS node for updating RViz UI
"""
class RvizUpdater():
    def __init__(self):  
        rospy.init_node('a_star_client_apo')

        # set up topics   
        self.subMap = rospy.Subscriber("/map", OccupancyGrid, self.saveOccupancyGrid, queue_size=1) # Callback function to load map on initial load                           
        self.pubExpanded = rospy.Publisher('/expanded', OccupancyGrid, queue_size=10)  
        rospy.Timer(rospy.Duration(2), self.publishGrid)               

        # set up control parameters        
        self.frameID = String()
        self.frameID.data = "map" 
        self.map = OccupancyGrid()
        rospy.spin()

    """
    expands obstacles and save grid for future use
    """
    def saveOccupancyGrid(self, grid):         
        self.map = padding.dilateByK(grid, PADDING)             
   
    """
    updates map in RViz constantly
    """
    def publishGrid(self, event):
        self.pubExpanded.publish(self.map)

    """
    creates PointStamped message given x and y
    """
    def createPointStamped(self, x, y):   
        point = PointStamped()
        point.header.seq = 1
        point.header.stamp = rospy.Time.now()
        point.header.frame_id = self.frameID.data
        point.point.x = x
        point.point.y = y        
        return point

# The program's primary executing section
if __name__ == '__main__':    
    RvizUpdater()
