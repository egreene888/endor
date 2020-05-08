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
from geometry_msgs.msg import Twist, Point
from std_msgs.msg import String

import numpy as np

MAX_SPEED = 1			  # Maximum linear velocity we can command our motors
MAX_TURN_RATE = 0.2		  # Maximum angular velocity we can command our motors

ACCEL_LINEAR = 0.1
ACCEL_ANGULAR = 0.05

CONTROL_PERIOD = rospy.Duration(0.1) # Control period of 10 Hz.

class controller(object):
	def __init__(self):
		# create the RosPy node
		rospy.init_node('controller')

		# current linear and angular velocity
		self.current_linear = 0
		self.current_angular = 0

		# desired linear and angular velocity-- may be different from current
		self.desired_linear = 0
		self.desired_angular = 0

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
		# TODO: Figure out what messages will come and how to handle them
		rospy.loginfo(msg)
		self.key_queue.append(msg)

		return

	def control_callback(self):
		"""
		The control callback is on a timer, so events will change the state of
		the motor, but only this callback will send commands to the motor. This
		way there is no chance of duplicate or conflicting messages
		"""
