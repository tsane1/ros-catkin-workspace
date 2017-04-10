#!/usr/bin/env python

""" 
ROS Server calculating the path between two points on a map using the A* algorithm

Authors: Matthew Piazza
Date: 4/10/17
"""

import rospy, tf
from std_msgs.msg import String
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import Point, Pose, Quaternion, PoseStamped, PointStamped
from tsane_mmlamare_mwpiazza_lab4.srv import *

# ROS node 
class AStarServiceClient():
    def __init__(self):  
        rospy.init_node('a_star_client_apo')

        # set up topics   
        self.subMap = rospy.Subscriber("/map", OccupancyGrid, self.saveOccupancyGrid, queue_size=1) # Callback function to load map on initial load                           
        self.subEnd = rospy.Subscriber("/customGoal", PoseStamped, self.setEnd, queue_size=1) # Callback function to load map on initial load
        self.pubEnd = rospy.Publisher('/endPoint', PointStamped, queue_size=10)      
        self.pubWaypoints = rospy.Publisher('/waypoints', Path, queue_size=10)     
        self._odom_list = tf.TransformListener() # Get the robot's Odometry
        rospy.Timer(rospy.Duration(.1), self.monitorOdometry)

        # set up control parameters        
        self.frameID = String()
        self.frameID.data = "map"         
        self.startIsSet = False
        self.endIsSet = False   
        rospy.spin()

    # Read the robot's current position
    def monitorOdometry(self, event):    
        self._odom_list.waitForTransform("odom", "base_footprint", rospy.Time(0), rospy.Duration(1.0))
        (pos, quaternion) = self._odom_list.lookupTransform("odom", "base_footprint", rospy.Time(0)) 
        self.startPose = Pose()        
        self.startPose.position = Point(pos[0], pos[1], pos[2])        
        self.startPose.orientation = Quaternion(quaternion[0], quaternion[1], quaternion[2], quaternion[3])
        self.startIsSet = True

    # save grid as an attribute to send in service
    def saveOccupancyGrid(self, grid):
        self.map = grid    

    # sets end of A* based on Publish Point
    def setEnd(self, poseStampedMsg):        
        self.goalPose = poseStampedMsg.pose
        self.pubEnd.publish(self.createPointStamped(self.goalPose.position.x, self.goalPose.position.y)) # send to RViz
        self.endIsSet = True        
        if self.startIsSet and self.endIsSet:
        	self.callAStar() 
        else:
        	print("Robot odometry unread")

    # run A* algorithm and send grid cells to RViz
    def callAStar(self):
        rospy.wait_for_service('a_star')
        try:
            aStarService = rospy.ServiceProxy('a_star', AStar)            
            response = aStarService(self.frameID, self.map, self.startPose, self.goalPose)
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