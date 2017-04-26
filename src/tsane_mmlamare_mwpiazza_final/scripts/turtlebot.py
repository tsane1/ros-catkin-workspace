#!/usr/bin/env python

"""
Code specific to driving a Turtlebot

Author:
    - Tanuj Sane
    - Matthew Lamare
    - Matthew Piazza

Since:
    - 4/8/2017

Version:
    - 1.0 Initial commit
"""

# Imports
import rospy, tf, math
from std_msgs.msg import String
from nav_msgs.msg import Path, OccupancyGrid
from geometry_msgs.msg import Twist, Pose, Point, PointStamped, PoseStamped, Quaternion
from tf.transformations import euler_from_quaternion
from tsane_mmlamare_mwpiazza_final.srv import *

#REPLAN_RATE = 2 # seconds
ODOM_RATE = .1 # seconds
REPLAN_INTERVAL = 10           # seconds
GOAL_TOLERANCE = .5            # meters
ROTATE_TOLERANCE = .1          # radians
STRAIGHT_SPEED = .15           # m/sec
STRAIGHT_BUFFER = .05          # number of cms to stop early
ROTATE_SPEED = .5              # rad/sec
USE_COSTMAP = False

"""
Encapsulation of a Turtlebot with necessary functionality
"""
class Turtlebot():
    """
    Constructor    
    """
    def __init__(self):
        rospy.init_node('turtlebot_apo')

        # constants
        self.spinWheelsInterval = .01 # seconds
        self.replanInterval = 60      # seconds
        self.frameID = String()
        self.frameID.data = "map"

        # State
        self.mapIsSet = False
        self.costMapIsSet = False
        self.startIsSet = False
        self.lastNavTime = rospy.Time.now()
        self.pose = Pose()
        self.map = OccupancyGrid()
        self.costMap = OccupancyGrid()

        # Publishers
        self.driver = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, queue_size=10)
        self.pubWaypoints = rospy.Publisher('/waypoints', Path, queue_size=10)
        self.pubPos = rospy.Publisher('/currentPos', PointStamped, queue_size=10)
        self.pubEnd = rospy.Publisher('/endPoint', PointStamped, queue_size=10)  
        
        # Subscribers            
        self.subMap = rospy.Subscriber("/expanded", OccupancyGrid, self.saveMap, queue_size=1)
        if USE_COSTMAP:
            self.subCostMap = rospy.Subscriber("/move_base/global_costmap/costmap", OccupancyGrid, self.saveCostMap, queue_size=1)

        # Timers
        self.odometry = tf.TransformListener()
        rospy.Timer(rospy.Duration(ODOM_RATE), self.monitorOdometry)
        rospy.sleep(rospy.Duration(1, 0)) # wait for a moment to set pose
        self.navigate()
    
    """
    Helper function to save expanded obstacle map to class
    """
    def saveMap(self, grid):
        self.map = grid        
        self.mapIsSet = True

    """
    Helper function to save costmap to class
    """
    def saveCostMap(self, grid):
        self.costMap = grid
        self.costMapIsSet = True   

    """
    Read the robot's current position
    """
    def monitorOdometry(self, event):    
        self.odometry.waitForTransform("odom", "base_footprint", rospy.Time(0), rospy.Duration(1.0))
        (pos, quaternion) = self.odometry.lookupTransform("odom", "base_footprint", rospy.Time(0))         
        self.pose.position = Point(pos[0], pos[1], pos[2])        
        orientation = Quaternion(quaternion[0], quaternion[1], quaternion[2], quaternion[3])        
        self.pose.orientation.z = tf.transformations.euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])[2]
        self.publishPosition()
        self.startIsSet = True    

    """
    navigate along a path of waypoints to the set end goal pose
    """
    def navigate(self):   
        #self.scanSurroundings() TODO   
        while True: 
            if self.startIsSet and self.mapIsSet and (self.costMapIsSet or not USE_COSTMAP):                
                self.scanSurroundings()
                path = self.callAStar()            
                if path == []:
                    print("TURTLEBOT: Goal Reached")
                    continue 
                self.lastNavTime = rospy.Time.now()
                for pose in path: 
                    self.navToPose(pose)
                print("TURTLEBOT: navigation iteration complete. replanning")

    """
    Asks the AStarService for a new plan    
    """
    def callAStar(self):
        rospy.wait_for_service('a_star')
        try:
            aStarService = rospy.ServiceProxy('a_star', AStar)                            
            response = aStarService(self.frameID, self.map, self.costMap, self.pose)            
            self.pubWaypoints.publish(response.waypoints)            
            if (len(response.waypoints.poses) > 0):
                self.publishGoal(response.waypoints.poses[-1].pose.position) 
            
            path = []             
            for poseStamped in response.waypoints.poses: 
                path.append(poseStamped.pose) 
            print("TURTLEBOT: A* Waypoints Received:", len(path))
            return path
        
        except rospy.ServiceException, e:
            print("TURTLEBOT: Service call failed:\n", e)

    """
    drive to a goal subscribed as /move_base_simple/navPose
    """
    def navToPose(self, goal):        
        # Inverse Kinematics        
        finalAngle = tf.transformations.euler_from_quaternion([goal.orientation.x, goal.orientation.y, goal.orientation.z, goal.orientation.w])[2]    
        xDiff = goal.position.x - self.pose.position.x
        yDiff = goal.position.y - self.pose.position.y      
        preturnAngle = math.atan2(yDiff, xDiff)
        dist = math.hypot(xDiff, yDiff)         
        # execute move       
        if dist > GOAL_TOLERANCE:
            print("TURTLEBOT: Naving to pose")
            self.rotateTo(preturnAngle)     # rotate towards goal
            self.driveStraightBy(dist)  # move to goal
            self.rotateTo(finalAngle)       # rotate towards final orientation 
        else:
            print("TURTLEBOT: Waypoint within goal tolerance")         

    """
    drive to a goal subscribed as /move_base_simple/navPose
    """
    def scanSurroundings(self):
        print("TURTLEBOT: Scanning surroundings")
        self.replanInterval = 1000
        self.lastNavTime = rospy.Time.now()        
        self.rotateTo(0)     
        rospy.sleep(.5)
        self.lastNavTime = rospy.Time.now()        
        self.rotateTo(math.pi*2/3)     
        rospy.sleep(.5)
        self.lastNavTime = rospy.Time.now()        
        self.rotateTo(-math.pi*2/3)
        rospy.sleep(.5)
        self.lastNavTime = rospy.Time.now()        
        self.rotateTo(0)
        self.replanInterval = REPLAN_INTERVAL
        print("TURTLEBOT: Finished scanning surroundings")

    """
    Drivers
    """

    """
    Accepts an orientation angle and makes the robot rotate to it.
    """
    def rotateTo(self, angle):
        if (angle <= math.pi and angle >= -math.pi): # if angle in valid bounds
            nearAngle = False  
            while not self.hasIntervalPassed(self.lastNavTime, self.replanInterval) and not nearAngle:
                # find necessary change in angle to turn by
                deltaAngle = self.pose.orientation.z - angle 
                deltaAngle = deltaAngle + 2*math.pi if (deltaAngle < -math.pi) else deltaAngle
                deltaAngle = deltaAngle - 2*math.pi if (deltaAngle > math.pi) else deltaAngle 
                speed = ROTATE_SPEED if deltaAngle < 0 else -ROTATE_SPEED       
                self.publishTwist(0, speed)
                  
                # check for replanning timeout and finishing single rotation        
                difference = abs(self.pose.orientation.z - angle)
                nearAngle = difference < ROTATE_TOLERANCE       
            self.publishTwist(0, 0)     
            if self.hasIntervalPassed(self.lastNavTime, self.replanInterval):
                print("TURTLEBOT: Rotation replanning timeout")    

        else: # angle not valid
            print("TURTLEBOT: Angle not within PI to negative PI bounds")

    """
    Accepts a speed and a distance for the robot to move in a straight line
    """
    def driveStraightBy(self, distance):   
        overDistance = False  
        originalPos = self.pose.position # copy origin
        while not self.hasIntervalPassed(self.lastNavTime, self.replanInterval) and not overDistance:
            self.publishTwist(STRAIGHT_SPEED, 0)
              
            # check for replanning timeout and driving past distance
            xDistance = self.pose.position.x - originalPos.x
            yDistance = self.pose.position.y - originalPos.y
            distanceTraveled = math.hypot(xDistance, yDistance)
            overDistance = distanceTraveled > (distance - STRAIGHT_BUFFER)
        self.publishTwist(0, 0)     
        if self.hasIntervalPassed(self.lastNavTime, self.replanInterval):
            print("TURTLEBOT: Rotation replanning timeout")     

    """
    Returns whether the given number of seconds have passed
    """
    def hasIntervalPassed(self, lastUpdate, intervalSec):
        currentTime = rospy.Time.now()
        timeElapsed = float(currentTime.secs - lastUpdate.secs)
        nsecElapsed = float(currentTime.nsecs - lastUpdate.nsecs)
        if nsecElapsed < 0:
            timeElapsed -= 1
        timeElapsed += nsecElapsed/1000000000.0       
        return timeElapsed > intervalSec

    """
    A helper for publishing Twist messages
    """
    def publishTwist(self, linear, angular):
        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        self.driver.publish(twist)

    """
    A helper for publishing PoseStamped messages
    """
    def publishPosition(self):   
        point = PointStamped()
        point.header.seq = 1
        point.header.stamp = rospy.Time.now()
        point.header.frame_id = self.frameID.data
        point.point.x = self.pose.position.x
        point.point.y = self.pose.position.y        
        self.pubPos.publish(point)

    """
    A helper for publishing PoseStamped messages
    """
    def publishGoal(self, position):   
        point = PointStamped()
        point.header.seq = 1
        point.header.stamp = rospy.Time.now()
        point.header.frame_id = self.frameID.data
        point.point.x = position.x
        point.point.y = position.y        
        self.pubEnd.publish(point)
                    

# The program's primary executing section
if __name__ == '__main__':    
    Turtlebot()
