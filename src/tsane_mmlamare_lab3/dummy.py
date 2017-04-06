#!/usr/bin/env python

"""
A dummy node that takes in start and goal positions, and returns and colors a path from the A* service
"""

# Imports
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from nav_msgs.srv import GetPlan

if __name__ == '__main__':
	rospy.init_node('dummy')
	rospy.wait_for_service('path_planner')

	start = PoseStamped()
	start.pose.position.x = input("Start X:")
	start.pose.position.y = input("Start Y:")
	
	goal = PoseStamped()
	goal.pose.position.x = input("Goal X:")
	goal.pose.position.y = input("Goal Y:")

	planner = rospy.ServiceProxy('path_planner', GetPlan)
	path_pub = rospy.Publisher('/a_star/path', Path, queue_size=0)

	try:
		path = planner(start, goal, 0.5)
		
	except rospy.ServiceException as e:
		print e

	else:
		print "Thanks!"
		
		for i in range(10):
			path_pub.publish(path.plan)
			rospy.sleep(rospy.Duration(0.5))

	rospy.spin()
