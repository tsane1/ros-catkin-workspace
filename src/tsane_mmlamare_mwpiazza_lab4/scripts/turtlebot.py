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
import rospy, tf, math, sys
from geometry_msgs.msg import Twist, Pose, PoseStamped
from kobuki_msgs.msg import BumperEvent
from tf.transformations import euler_from_quaternion

"""
Encapsulation of a Turtlebot with necessary functionality
"""

class Turtlebot:
	"""
	Constructor	
	"""
	def __init__(self):
		# Kobuki base constants
		self._r = 0.035		# wheel radius, m
		self._L = 0.230		# wheelbase, m

		# State
		self.pose = Pose()
		self.odometry = tf.TransformListener()

		# Publishers
		self.driver = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, queue_size = 10)
		
		# Subscribers	
		self.bumper = rospy.Subscriber('/mobile_base/events/bumper', BumperEvent, self.on_bump, queue_size = 1)
		self.nav = rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.on_nav)

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
	Event handlers
	"""
	def on_bump(msg):
		if msg.state == 1:
			print "Bump!"

	def on_nav(msg):
		print msg

	def on_tick(self, msg):
		try:
			self.odometry.waitForTransform('odom', 'base_footprint', rospy.Time(0), rospy.Duration(1.0))
			(position, orientation) = self.odometry.lookupTransform('odom','base_footprint', rospy.Time(0))
		
		except: raise

		# Update state
		self.pose = Pose()

		self.pose.position.x = position[0]
		self.pose.position.y = position[1]
		self.pose.position.z = position[2]

		dummy = orientation
		q = [dummy[0], dummy[1], dummy[2], dummy[3]]							# put the angular quaternion into a list
		roll, pitch, yaw = euler_from_quaternion(q)

		self.pose.orientation.z = yaw

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

			if d_t >= distance:						# Traveled the necessary distance
				arrived = True
				self.publish_twist(0, 0)		# STOP!
			else:
				self.publish_twist(speed, 0)
	
	def rotate(self, angle):
			# Set wheel speeds
			w_right = (angle / abs(angle)) * 5		# randomly chosen wheelspeed of 5 rad/sec
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

				if abs(theta_remaining) <= 0.3:		# Turned the necessary angle within error
					arrived = True
					self.publish_twist(0, 0)				# STOP!
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
	
		theta_g = abs(math.atan2((y_f - y_o), (x_f - x_o)) - theta_o)		# angle between current orientation and new position
		d_t = math.sqrt((x_f - x_o) ** 2 + (y_f - y_o) ** 2)						# how far to go in that direction
		
		dummy = goal.orientation
		q = [dummy.x, dummy.y, dummy.z, dummy.w]												# put the angular quaternion into a list
		roll, pitch, yaw = euler_from_quaternion(q)
		theta_f = yaw - (theta_g + theta_o)															# complete the turn to face the specified pose	

		self.rotate(theta_g)
		self.drive_straight(0.25, d_t)
		self.rotate(theta_f)

