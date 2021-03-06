#!/usr/bin/env python

"""	
The main file for RBE 3002 Lab 4 at Worcester Polytechnic Institute

Authors:
	- Tanuj Sane
	- Matthew Lamare
	- Matthew Piazza

Since:
	- 4/6/2017

Version:
	- 1.0 Initial commit
"""

# Imports
import rospy
from callbacks import AsyncValue
from turtlebot import Turtlebot
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from nav_msgs.srv import GetPlan

# Main
if __name__ == '__main__':
	# Start the node!
	rospy.init_node('tsane_mmlamare_mwpiazza_lab4')
	rospy.wait_for_service('path_planner')

	# Setup
	turtlebot = Turtlebot()
	planner = rospy.ServiceProxy('path_planner', GetPlan)

	start = PoseStamped()
	start.pose.position.x = input("Start X: ")
	start.pose.position.y = input("Start Y: ")
	
	goal = PoseStamped()
	goal.pose.position.x = input("Goal X: ")
	goal.pose.position.y = input("Goal Y: ")

	# Start the timer
	rospy.Timer(rospy.Duration(0.1), turtlebot.on_tick)

	try:
		path = planner(start, goal, 0.5)
		
	except rospy.ServiceException as e:
		print e

	else:
		for posestamped in path.plan.poses:
			print 'moving'
			turtlebot.nav_to_pose(posestamped.pose)

	rospy.spin()
