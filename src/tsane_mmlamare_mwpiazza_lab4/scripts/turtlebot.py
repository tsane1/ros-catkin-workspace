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
from geometry_msgs.msg import Twist, Pose, Point, PoseStamped, Quaternion
from tf.transformations import euler_from_quaternion
from tsane_mmlamare_mwpiazza_lab4.srv import *

#REPLAN_RATE = 2 # seconds
ODOM_RATE = .1 # seconds
GOAL_TOLERANCE = .5 # meters
STRAIGHT_SPEED = .2      # m/sec
STRAIGHT_SPEED_FACTOR = 1    # multiplication factor to achieve m/sec units
ROTATE_SPEED_FACTOR = .84       # multiplication factor to achieve rad/sec units
ACCELERATION_FACTOR = .75

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
        self.spinWheelsInterval = 100000000 # .1 seconds
        self.replanInterval = 5             # seconds
        self.frameID = String()
        self.frameID.data = "map" 

        # State
        self.mapIsSet = False
        self.costMapIsSet = False
        self.startIsSet = False
        self.endIsSet = False
        self.pose = Pose()

        self.map = OccupancyGrid()

        # Publishers
        self.driver = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, queue_size=10)
        self.pubWaypoints = rospy.Publisher('/waypoints', Path, queue_size=10)
        
        # Subscribers            
        self.subMap = rospy.Subscriber("/expanded", OccupancyGrid, self.saveMap, queue_size=1)
        self.subCostMap = rospy.Subscriber("/move_base/global_costmap/costmap", OccupancyGrid, self.saveCostMap, queue_size=1)
        self.subEnd = rospy.Subscriber("/customGoal", PoseStamped, self.setEndAndNav, queue_size=1)

        # Timers
        self.odometry = tf.TransformListener()
        rospy.Timer(rospy.Duration(ODOM_RATE), self.monitorOdometry)
        rospy.spin()
    
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
    Helper function to set goal and navigate to it using A*
    """
    def setEndAndNav(self, poseStampedMsg):
        self.goalPose = poseStampedMsg.pose
        self.endIsSet = True
        if self.startIsSet and self.mapIsSet and self.endIsSet and self.costMapIsSet:
            self.navigate() 
        elif not self.startIsSet or not self.endIsSet:
            print("TURTLEBOT: Endpoints not set")
        else:
            print("TURTLEBOT: Map unknown")   

    """
    Read the robot's current position
    """
    def monitorOdometry(self, event):    
        self.odometry.waitForTransform("odom", "base_footprint", rospy.Time(0), rospy.Duration(1.0))
        (pos, quaternion) = self.odometry.lookupTransform("odom", "base_footprint", rospy.Time(0))         
        self.pose.position = Point(pos[0], pos[1], pos[2])        
        orientation = Quaternion(quaternion[0], quaternion[1], quaternion[2], quaternion[3])        
        self.pose.orientation.z = tf.transformations.euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])[2]
        self.startIsSet = True    

    """
    navigate along a path of waypoints to the set end goal pose
    """
    def navigate(self):               
        while True:              
            path = self.callAStar()            
            self.lastNavTime = rospy.Time.now()
            print("TURTLEBOT: Waypoints left to navigate:", len(path))
            if path == []:
                break 
            for pose in path: 
                self.navToPose(pose)

    """
    Asks the AStarService for a new plan    
    """
    def callAStar(self):
        rospy.wait_for_service('a_star')
        try:
            aStarService = rospy.ServiceProxy('a_star', AStar)                            
            response = aStarService(self.frameID, self.map, self.costMap, self.pose, self.goalPose)            
            self.pubWaypoints.publish(response.waypoints)
            
            # reverse waypoints into correct order
            path = []             
            for poseStamped in response.waypoints.poses: 
                path.insert(0, poseStamped.pose)  
            return path[1:]
        
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
            self.rotateTo(preturnAngle)     # rotate towards goal
            self.driveStraightBy(STRAIGHT_SPEED, dist)  # move to goal
            self.rotateTo(finalAngle)       # rotate towards final orientation           

    """
    Drivers
    """

    """
    Accepts an orientation angle and makes the robot rotate to it.
    """
    def rotateTo(self, angle):
        if (angle < math.pi and angle > -math.pi): # if angle in valid bounds
            # find necessary change in angle to turn by
            deltaAngle = self.pose.orientation.z - angle 
            deltaAngle = deltaAngle + 2*math.pi if (deltaAngle < -math.pi) else deltaAngle
            deltaAngle = deltaAngle - 2*math.pi if (deltaAngle > math.pi) else deltaAngle 
            self.rotateBy(deltaAngle)  
        else: # angle not valid
            print("TURTLEBOT: Angle not within PI to negative PI bounds")

    """
    Accepts a speed and a distance for the robot to move in a straight line
    """
    def driveStraightBy(self, speed, distance):        
        time = distance*STRAIGHT_SPEED_FACTOR/float(speed)
        """
        # accelerate
        for factor in [.25, .5, .75]: # 3 part acceleration before reaching full power 
            self.spinWheels(speed*factor, speed*factor, time/12.0)
        # normal movement
        self.spinWheels(speed, speed, time*ACCELERATION_FACTOR)        
        # decelerate
        for factor in [.75, .5, .25]: # 3 part deceleration before stopping
            self.spinWheels(speed*factor, speed*factor, time/12.0)
        """ 
        self.spinWheels(speed, speed, time)

    """
    Accepts an angle and makes the robot rotate by that amount.
    """
    def rotateBy(self, angle): 
        speed = ROTATE_SPEED_FACTOR if angle > 0 else -ROTATE_SPEED_FACTOR        
        seconds = abs(angle*math.pi/2.0) # turn 90 degrees per second              
        self.spinWheels(-speed, speed, seconds)

    """
    Converts wheel velocities and time into control commands for the robot
    """
    def spinWheels(self, velLeft, velRight, time):
        timeSec = time//1   
        timeNSec = (time-timeSec)*1000000000
        spinTimePassed = False
        timeStart = rospy.Time.now()
        timeLastUpdate = timeStart        
        # check for replanning timeout and finishing single rotation
        while not self.hasSecIntervalPassed(self.lastNavTime, self.replanInterval) and not spinTimePassed:
            if self.hasNSecIntervalPassed(timeLastUpdate, self.spinWheelsInterval):  
                # rotate           
                linear, angular = self.convertLinAng(velLeft, velRight)
                self.publishTwist(linear, angular)   
                timeLastUpdate = rospy.Time.now()        
                # check for finishing single rotation
                secPassed = self.hasSecIntervalPassed(timeStart, timeSec)
                nsecPassed = self.hasNSecIntervalPassed(timeStart, timeNSec)            
                spinTimePassed = secPassed and nsecPassed    
        if self.hasSecIntervalPassed(self.lastNavTime, self.replanInterval):
            print("Replanning timeout")                

    """
    Converts wheel velocities to linear and angular commands
    """
    def convertLinAng(self, velLeft, velRight):
        linear = 0
        angular = 0
        if (velLeft == velRight): # move straight
            linear = velLeft
        elif (velLeft == -velRight): # rotate
            angular = velLeft        
        return linear, angular   

    """
    Returns whether the given number of seconds have passed
    """
    def hasSecIntervalPassed(self, lastUpdate, intervalSec):
        currentTime = rospy.Time.now()
        secElapsed = currentTime.secs - lastUpdate.secs
        nsecElapsed = currentTime.nsecs - lastUpdate.nsecs        
        if nsecElapsed < 0:
            secElapsed -= 1        
        return secElapsed > intervalSec

    """
    Returns whether the given number of nanoseconds have passed.
    Works for periods less than one second
    """
    def hasNSecIntervalPassed(self, lastUpdate, intervalNSec):
        currentTime = rospy.Time.now()
        secElapsed = currentTime.secs - lastUpdate.secs
        nsecElapsed = currentTime.nsecs - lastUpdate.nsecs
        if nsecElapsed < 0:
            secElapsed -= 1
            nsecElapsed += 1000000000
        return nsecElapsed > intervalNSec

    """
    A helper for publishing Twist messages
    """
    def publishTwist(self, linear, angular):
        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        self.driver.publish(twist)
                    

# The program's primary executing section
if __name__ == '__main__':    
    Turtlebot()
