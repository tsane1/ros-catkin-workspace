#!/usr/bin/env python

""" 
ROS Server calculating the path between two points on a map using the A* algorithm

Author:
    - Tanuj Sane
    - Matthew Lamare
    - Matthew Piazza

Date: 4/10/17
"""

import rospy, tf
from std_msgs.msg import String
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped, PointStamped
import padding

PADDING = .3 # meters

# ROS node 
class AStarServiceClient():
    def __init__(self):  
        rospy.init_node('a_star_client_apo')

        # set up topics   
        self.subMap = rospy.Subscriber("/map", OccupancyGrid, self.saveOccupancyGrid, queue_size=1) # Callback function to load map on initial load                           
        #self.subEnd = rospy.Subscriber("/customGoal", PoseStamped, self.setEnd, queue_size=1) # Callback function to load map on initial load    
        self.pubExpanded = rospy.Publisher('/expanded', OccupancyGrid, queue_size=10)  
        rospy.Timer(rospy.Duration(2), self.publishGrid)               

        # set up control parameters        
        self.frameID = String()
        self.frameID.data = "map" 
        self.map = OccupancyGrid()
        rospy.spin()

    # expands obstacles and save grid for future use
    def saveOccupancyGrid(self, grid):         
        self.map = padding.dilateByK(grid, PADDING)             

    # sets end of A* based on Publish Point
    def setEnd(self, poseStampedMsg):      
        self.pubEnd.publish(self.createPointStamped(poseStampedMsg.pose.position.x, poseStampedMsg.pose.position.y)) # send to RViz

    # updates map in RViz constantly
    def publishGrid(self, event):
        self.pubExpanded.publish(self.map)

    # create PointStamped message given x and y
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
    AStarServiceClient()
