#!/usr/bin/env python

""" 
ROS Server calculating the path between two points on a map using the A* algorithm

Authors: Joseph Lombardi, Matthew Piazza
Date: 4/5/17
"""

import rospy
from std_msgs.msg import String
from nav_msgs.msg import GridCells, OccupancyGrid, Path
from geometry_msgs.msg import Point, PoseWithCovarianceStamped, PointStamped
from mpiazza_jlombardi_lab3.srv import *

# ROS node 
class AStarServiceClient():
    def __init__(self):  
        rospy.init_node('a_star_client_mpiazza_jlombardi')

        # set up topics   
        self.subMap = rospy.Subscriber("/map", OccupancyGrid, self.saveOccupancyGrid, queue_size=1) # Callback function to load map on initial load                           
        self.subStart = rospy.Subscriber("/initialpose", PoseWithCovarianceStamped, self.setStart, queue_size=1) # Callback function to load map on initial load        
        self.subEnd = rospy.Subscriber("/clicked_point", PointStamped, self.setEnd, queue_size=1) # Callback function to load map on initial load
        self.pubStart = rospy.Publisher('/startPoint', PointStamped, queue_size=10)    
        self.pubEnd = rospy.Publisher('/endPoint', PointStamped, queue_size=10)  
        self.pubFrontier = rospy.Publisher('/frontier', GridCells, queue_size=10) 
        self.pubVisited = rospy.Publisher('/visited', GridCells, queue_size=10) 
        self.pubPath = rospy.Publisher('/path', GridCells, queue_size=10)         
        self.pubWaypoints = rospy.Publisher('/waypoints', Path, queue_size=10)         

        # set up control parameters        
        self.frameID = String()
        self.frameID.data = "map"         
        self.startIsSet = False
        self.endIsSet = False                    
        rospy.spin()

    # save grid as an attribute to send in service
    def saveOccupancyGrid(self, grid):
        self.map = grid

    # sets start of A* based on 2D Pose Estimate
    def setStart(self, poseMsg):
        self.startCoord = Point()
        self.startCoord.x = poseMsg.pose.pose.position.x
        self.startCoord.y = poseMsg.pose.pose.position.y
        self.startCoord.z = 0
        self.pubStart.publish(self.createPointStamped(self.startCoord.x, self.startCoord.y))
        self.startIsSet = True
        if self.startIsSet and self.endIsSet:
            self.callAStar()
        else:
        	print("Please set end point with RViz Publish Point tool")       

    # sets end of A* based on Publish Point
    def setEnd(self, pointStampedMsg):
        self.goalCoord = Point()
        self.goalCoord.x = pointStampedMsg.point.x
        self.goalCoord.y = pointStampedMsg.point.y
        self.goalCoord.z = 0
        self.pubEnd.publish(self.createPointStamped(self.goalCoord.x, self.goalCoord.y))
        self.endIsSet = True
        if self.startIsSet and self.endIsSet:
        	self.callAStar() 
        else:
        	print("Please set start point with RViz 2D Pose Estimate tool")       

    # run A* algorithm and send grid cells to RViz
    def callAStar(self):
        rospy.wait_for_service('a_star')
        try:
            aStarService = rospy.ServiceProxy('a_star', AStar)            
            response = aStarService(self.frameID, self.map, self.startCoord, self.goalCoord)
            self.pubFrontier.publish(response.frontier)
            self.pubVisited.publish(response.visited)
            self.pubPath.publish(response.path)
            self.pubWaypoints.publish(response.waypoints)       

        except rospy.ServiceException, e:
            print "Service call failed: %s"%e  	    	

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