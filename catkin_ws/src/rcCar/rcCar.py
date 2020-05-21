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

		# Will publish out velocity to writer node
		# Using topic name "key_vel"
		self.cmd_vel_pub = rospy.Publisher('key_vel', Twist)

		# create a subscriber to the key_teleop service.
		rospy.Subscriber('key_teleop', String , self.key_callback)
		# Timer for updating controller
		rospy.Timer(CONTROL_PERIOD, self.control_callback)

		# create a key_queue which will keep track of all the keys that have
		# been pressed since the last control callback
		self.key_queue = []
		self.run()

	def key_callback(self, msg):
		"""
		The key callback is called whenever a message comes in from the
		key_teleop node. It will add the key press to the queue.
		"""
		# TODO: Figure out what messages will come and how to handle them
		rospy.loginfo(msg)
		self.key_queue.append(msg)
		return

	def control_callback(self, timer_event=None):
		"""
		The control callback is on a timer, so events will change the state of
		the motor, but only this callback will send commands to the motor. This
		way there is no chance of duplicate or conflicting messages
		"""
		# read through the key_queue and determine if up and down are in there
		# if they are both there, they cancel each other out, so do nothing.
		if ("UP" in self.key_queue) and ("DOWN" in self.key_queue):
			pass
		# If we want to speed up, use an exponetial response.
		elif "UP" in self.key_queue:
			self.linear_vel += (ACCEL_LINEAR *
				(2.71828 ** abs(self.linear_vel)))
		# If we want to slow down, use the same exponential response
		elif "DOWN" in self.key_queue:
			self.linear_vel -= (ACCEL *
				(2.71828 ** abs(self.linear_vel)))
		# clamp the desired velocity to the min and max
		self.linear_vel = min(max(self.linear_vel, -MAX_SPEED), MAX_SPEED)

		# Give the same treatment to the angular velocity.
		if ("RIGHT" in self.key_queue) and ("LEFT" in self.key_queue):
			pass
		elif "UP" in self.key_queue:
			self.angular_vel += (ACCEL_ANGULAR *
				(2.71828 ** abs(self.angular_vel)))
		elif "DOWN" in self.key_queue:
			self.angular_vel -= (ACCEL_ANGULAR *
				(2.71828 ** abs(self.angular_vel)))
		self.angular_vel = min(max(self.angular_vel, -MAX_TURN_RATE), MAX_TURN_RATE)

		# Use the "S" key as an emergency stop
		if "S" in self.key_queue:
			self.linear_vel = 0
			self.angular_vel = 0

		# Now publish the velocities.
		cmd_vel = Twist()
		self.cmd_vel.linear.x = self.linear_vel
		self.cmd_vel.angular.z = self.angular_vel
		self.cmd_vel_pub.publish(cmd_vel)
		return

	def run(self):
		# timers and callbacks are already set up, so just spin.
		# if spin returns we were interrupted by Ctrl+C or shutdown
		rospy.spin()
		return

if __name__ == "__main__":
	c = controller()
	controller.run()
