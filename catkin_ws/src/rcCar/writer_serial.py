#! /usr/bin/env python
"""
writer_serial.py

Written by Evan Greene
2019-12-02

For use commanding a Sabertooth 2x5 in simplified serial mode.
Subscribes to the geometry_msgs.msg.Twist ROS topic and uses that to control
the robot.
"""

import rospy
from geometry_msgs.msg import Twist
import serial, time

ENABLE_LOGGING = False

class writer(object):

	def __init__(self):
		# create a port.
		# some robustness, so it can run on both mac and linux
		try: # should work on linux
			# self.port = serial.Serial('/dev/cu.usbserial')
			self.port = serial.Serial('/dev/ttyUSB0',
				baudrate = 9600,
				bytesize = serial.EIGHTBITS,
				parity = serial.PARITY_NONE,
				stopbits = serial.STOPBITS_ONE)
		except serial.serialutil.SerialException: # should work on mac
			self.port = serial.Serial('/dev/tty.usbserial-A6033KY3',
				baudrate = 9600,
				bytesize = serial.EIGHTBITS,
				parity = serial.PARITY_NONE,
				stopbits = serial.STOPBITS_ONE)

		# initialize the values of angular_vel and linear_vel
		self.linear_vel = 0
		self.angular_vel = 0
		self.right_vel = 0
		self.left_vel = 0

		# Write initial data
		self.write()
		
		# create the rospy node
		rospy.init_node("Controller")
		
		# create the rospy subscriber.
		rospy.Subscriber('key_vel', Twist, self.update_vel)

	def update_vel(self, msg):
		msg.angular.z *= -1
		self.ramp_linear(int(0.5 * 127 * msg.linear.x))
		self.ramp_angular(0.5 * 127 * msg.angular.z)
		rospy.loginfo("Linear velocity: {} \t Angular velocity: {}\n".format(
			msg.linear.x, msg.angular.z))
		self.write()


	def write(self):
		"""calculates the values to send to the left and right motors
		then sends those values.
		"""
		"""
		Motor 1 -- 1 	is full reverse,  64 is stop, and 127 is full forward
		Motor 2 -- 128 	is full reverse, 192 is stop, and 255 is full forward
		Sending a value of zero stops both motors.
		"""
		# stop condition
		if not(self.linear_vel or self.angular_vel):
			self.left_vel = 0
			self.right_vel = 0
		else:
			# left motor is motor1
			self.left_vel = (self.linear_vel - self.angular_vel) + 64
			self.right_vel = (self.linear_vel + self.angular_vel)  + 192

		self.port.write([self.left_vel])
		self.port.write([self.right_vel])
		if ENABLE_LOGGING:
			rospy.loginfo("Commanded velocities: ")
			rospy.loginfo("Left: {}".format(self.left_vel))
			rospy.loginfo("Right: {}".format(self.right_vel))

	def send_linear_vel(self, vel):
		"""
		Takes a velocity as a signed integer between -63 and 63.
		Negative values are backwards, zero is stop, and positive values
		are forward.
		"""
		# the type checking is somewhat rudimentary.
		try:
			vel = int(vel)
			assert(-64 < vel and 64 > vel)
		except (TypeError, AssertionError):
			print('That is not a valid input')

		# all that's needed is to command the velocity.
		self.linear_vel = vel
		self.write()

	def send_angular_vel(self, vel):
		"""
		takes an angular velocity as a signed integer in the range (-63, 63).
		Negative results are right, zero is stop and positive values are
		left turns.
		"""
		# the type checking is somewhat rudimentary.
		try:
			vel = int(vel)
			assert(-128 < vel and 128 > vel)
		except (TypeError, AssertionError):
			print("That is not a valid input")

		# all that's needed is to command the velocity
		self.angular_vel = vel
		self.write()

	def stop(self):
		"""
		There are two ways to stop -- send values of 64 and 192 to
		motors 1 and 2, respectively, or send a value of zero.
		This method does both.
		"""
		self.linear_vel = 0
		self.angular_vel = 0
		self.write()

		# if that doesn't work this will.
		self.port.write([0])

	def ramp_linear(self, vel, rampTime):
		"""
		Ramps the linear velocity from the current level to the desired level
		over the time specified as the input (in seconds).
		"""
		if vel > self.linear_vel:
			velocities = range(self.linear_vel, vel)
		else:
			velocities = range(self.linear_vel, vel, -1)

		for velocity in velocities:
			self.send_linear_vel(velocity)
			time.sleep(rampTime / len(velocities))
		return

	def ramp_angular(self, vel, rampTime):
		"""
		Ramps the linear velocity from the current level to the desired level
		over the time specified as the input (in seconds).
		"""
		if vel > self.angular_vel:
			velocities = range(self.angular_vel, vel)
		else:
			velocities = range(self.angular_vel, vel, -1)
		for velocity in velocities:
			self.send_angular_vel(velocity)
			time.sleep(rampTime / len(velocities))
		return

	def run(self):
		rospy.spin()

	def test(self):
		print("Accelerating")
		self.ramp_angular(64, 2)
		print("Constant velocity")
		time.sleep(2)

		print("Decelerating")
		self.ramp_angular(0, 2)
		time.sleep(2)

		print("Stopping")
		self.stop()

def main():
	try:
		controller = writer()
		controller.run()
	except rospy.ROSInterruptException:
		pass

if __name__ == "__main__":
	main()
