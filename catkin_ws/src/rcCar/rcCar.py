#! /usr/bin/env python

"""
rcCar.py

Written by Evan Greene
2020-04-14

Uses ROS to create an RC car out of a robot. Uses the writer_serial object
to command the desired velocity.
"""

import roslib
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String

MAX_SPEED = 1			  # Maximum linear velocity we can command our motors
MAX_TURN_RATE = 0.2		  # Maximum angular velocity we can command our motors

ACCEL_LINEAR = 0.02
ACCEL_ANGULAR = 0.01

CONTROL_PERIOD = rospy.Duration(0.1) # Control period of 10 Hz.

class controller(object):
	def __init__(self):
		# create the RosPy node
		rospy.init_node('controller')

		# current linear and angular velocity
		self.linear_vel = 0
		self.angular_vel = 0

		# A key_queue to keep track of the inputs.
		self.key_queue = []

		# create a subscriber to get the input from the key_teleop node.
		self.key_vel_sub = rospy.Subscriber('key_vel', Twist, self.key_callback,
			queue_size = 10)

		# create a publisher to send the output to the writer_serial node.
		self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size = 10)

		# Create a timer for updating the commanded velocities.
		rospy.Timer(CONTROL_PERIOD, self.control_callback)

		# Nothing left to do but spin.
		self.run()

	def key_callback(self, msg):
		"""
		The key callback is called whenever a message comes in from the
		key_teleop node. It will add the key press to the queue.
		"""
		# Parse the msg.linear.x and add strings to the key_queue accordingly.
		if msg.linear.x > 0.01: # floating point error defense
			# This condition means the "UP" key is pressed.
			self.key_queue.append("UP")
		elif msg.linear.x < -0.01:
			self.key_queue.append("DOWN")

		# do the same for the msg.angular.z
		if msg.angular.z > 0.01:
			# Positive angular velocity corresponds to a left turn.
			self.key_queue.append("LEFT")
		elif msg.angular.z < -0.01:
			self.key_queue.append("RIGHT")
		return

	def control_callback(self, timer_event=None):
		"""
		The control callback is on a timer, so events will change the state of
		the motor, but only this callback will send commands to the motor. This
		way there is no chance of duplicate or conflicting messages
		"""
		# read through the key_queue and determine if up and down are in there
		# if they are both there, they cancel each other out, so let
		# artificial intertia slow the robot down.
		if ("UP" in self.key_queue) and ("DOWN" in self.key_queue):
			new_linear_vel = 0
		# If we want to speed up,
		elif "UP" in self.key_queue:
			new_linear_vel = 1
		# If we want to slow down,
		elif "DOWN" in self.key_queue:
			new_linear_vel = -1
		else:
			new_linear_vel = 0

		linear_weight = 0.1
		# The linear_vel is the moving average over the last few control periods
		self.linear_vel = self.linear_vel * (1 - linear_weight) + \
			new_linear_vel * linear_weight
		# clamp the desired velocity to the min and max
		self.linear_vel = min(max(self.linear_vel, -MAX_SPEED), MAX_SPEED)

		# Give the same treatment to the angular velocity.
		if ("RIGHT" in self.key_queue) and ("LEFT" in self.key_queue):
			new_angular_vel = 0
		elif "LEFT" in self.key_queue:
			new_angular_vel = 1
		elif "RIGHT" in self.key_queue:
			new_angular_vel = -1
		else:
			new_angular_vel = 0

		angular_weight = 0.1
		# The angular vel is the moving average over the last ten control
		# periods
		self.angular_vel = self.angular_vel * (1 - angular_weight)  + \
			new_angular_vel * angular_weight

		# clamp the velocity to the desired min and max. 
		self.angular_vel = min(max(self.angular_vel, -MAX_TURN_RATE), MAX_TURN_RATE)

		# Use the "S" key as an emergency stop
		if "S" in self.key_queue:
			self.linear_vel = 0
			self.angular_vel = 0

		# Now publish the velocities.
		cmd_vel = Twist()
		cmd_vel.linear.x = self.linear_vel
		cmd_vel.angular.z = self.angular_vel

		# Add some logging
		rospy.loginfo("Linear: {} \t Angular:{}".format(cmd_vel.linear.x,
			cmd_vel.angular.z))

		# clear the key_queue so commands don't get repeated.
		self.key_queue = []

		# publish the velocity we've commanded.
		self.cmd_vel_pub.publish(cmd_vel)
		return

	def run(self):
		# timers and callbacks are already set up, so just spin.
		# if spin returns we were interrupted by Ctrl+C or shutdown
		rospy.spin()
		return

if __name__ == "__main__":
	c = controller()
	c.run()
