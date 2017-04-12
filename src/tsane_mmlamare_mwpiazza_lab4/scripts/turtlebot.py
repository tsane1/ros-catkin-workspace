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

REPLAN_RATE = 2 # seconds
ODOM_RATE = .1 # seconds

"""
Encapsulation of a Turtlebot with necessary functionality
"""
class Turtlebot():
    """
    Constructor    
    """
    def __init__(self):
        rospy.init_node('turtlebot_apo')

        # Kobuki base constants
        self._r = 0.035        # wheel radius, m
        self._L = 0.230        # wheelbase, m
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
        self.subEnd = rospy.Subscriber("/customGoal", PoseStamped, self.setEnd, queue_size=1)

        # Timers
        self.odometry = tf.TransformListener()
        rospy.Timer(rospy.Duration(ODOM_RATE), self.monitorOdometry)
        rospy.Timer(rospy.Duration(REPLAN_RATE), self.replanPath) 

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
    Helper function to call A* with proper poses
    """
    def setEnd(self, poseStampedMsg):
        self.goalPose = poseStampedMsg.pose
        self.endIsSet = True

    def replanPath(self, timerEvent):
        if self.startIsSet and self.mapIsSet and self.endIsSet and self.costMapIsSet:
            self.callAStar()
        elif not self.startIsSet or not self.endIsSet:
            print("Endpoints not set")
        else:
            print("Map unknown")

    """
    Asks the AStarService for a new plan    
    """
    def callAStar(self):
        rospy.wait_for_service('a_star')
        try:
            aStarService = rospy.ServiceProxy('a_star', AStar)                    
            response = aStarService(self.frameID, self.map, self.costMap, self.pose, self.goalPose)
            self.pubWaypoints.publish(response.waypoints)
            self.navigate(response.waypoints)
        
        except rospy.ServiceException, e:
            print("Service call failed:\n", e)

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
    A helper for publishing Twist messages
    """
    def publish_twist(self, v_robot, w_robot):
        global teleop_pub

        # Build message
        twist = Twist()
        twist.linear.x = v_robot
        twist.angular.z = w_robot

        self.driver.publish(twist)

    """
    navigate with the given path
    """
    def navigate(self, path): 
        waypoints = []
        for poseStamped in path.poses: # reverse waypoints into correct order
            waypoints.insert(0, poseStamped.pose)        
        for pose in waypoints[1:]: # first way point is current position, ignore            
            pass
            #self.nav_to_pose(pose)

    """
    Drivers
    """
    def drive_straight(self, speed, distance):
        # Get starting location
        x_o = self.pose.position.x
        y_o = self.pose.position.y

        # Arrival flag - True when the robot has traveled the distance
        arrived = False
    
        while not arrived and not rospy.is_shutdown():
            # Get instantaneous position
            x_t = self.pose.position.x
            y_t = self.pose.position.y

            d_t = math.sqrt((x_t - x_o) ** 2 + (y_t - y_o) ** 2)

            if d_t >= distance:                        # Traveled the necessary distance
                arrived = True
                self.publish_twist(0, 0)        # STOP!
            else:
                self.publish_twist(speed, 0)
    
    def rotate(self, angle):
            # Set wheel speeds
            w_right = (angle / abs(angle)) * 5        # randomly chosen wheelspeed of 5 rad/sec
            w_left = -1 * w_right

            # Forward velocity kinematics
            w_robot = (self._r / self._L) * (w_right - w_left)

            # Get the initial angle
            theta_o = self.pose.orientation.z
    
            # Arrival flag - True when robot has turned the proper amount
            arrived = False

            while not arrived and not rospy.is_shutdown():
                # Get instantaneous angle
                theta_t = self.pose.orientation.z
    
                theta_turnt = abs(theta_t - theta_o)
                theta_remaining = abs(angle) - theta_turnt

                if abs(theta_remaining) <= 0.3:        # Turned the necessary angle within error
                    arrived = True
                    self.publish_twist(0, 0)           # STOP!
                else:
                    self.publish_twist(0, w_robot)

    def nav_to_pose(self, goal):        
        # Get current data
        x_o = self.pose.position.x
        y_o = self.pose.position.y
        theta_o = self.pose.orientation.z
    
        # Get goal data
        x_f = goal.position.x
        y_f = goal.position.y
    
        theta_g = abs(math.atan2((y_f - y_o), (x_f - x_o)) - theta_o)        # angle between current orientation and new position
        d_t = math.sqrt((x_f - x_o) ** 2 + (y_f - y_o) ** 2)                 # how far to go in that direction
        
        dummy = goal.orientation
        q = [dummy.x, dummy.y, dummy.z, dummy.w]                             # put the angular quaternion into a list
        roll, pitch, yaw = euler_from_quaternion(q)
        theta_f = yaw - (theta_g + theta_o)                                  # complete the turn to face the specified pose    

        self.rotate(theta_g)
        self.drive_straight(0.25, d_t)
        self.rotate(theta_f)

# The program's primary executing section
if __name__ == '__main__':    
    Turtlebot()
