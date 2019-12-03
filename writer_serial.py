"""
writer_serial.py

Written by Evan Greene 
2019-12-02

For use commanding a Sabertooth 2x5 in simplified serial mode.

"""
import serial, time

class writer(object):

    def __init__(self):
        # create a port.
        # some robustness, so it can run on both mac and linux
        try: # should work on linux
            # self.port = serial.Serial('/dev/cu.usbserial')
            self.port = serial.Serial('/dev/ttyUSB0')
        except serial.serialutil.SerialException: # should work on mac
            self.port = serial.Serial('/dev/cu.usbserial')
	# set the baud rate for the port.
	self.port.baudrate = 9600

	# set up some fields for the values we're sending. 
	self.angular_vel = 0
	self.linear_vel = 0
	self.left_vel = 0
	self.right_vel = 0

        # set up a timeout, where the motor will time out after not
        # recieving a command for five seconds.
        self.write()




    def write(self):
	"""calculates the values to send to the left and right motors 
	then sends those values. 
	"""
	"""
	The documentation says "sending a value between 1-127 will command 
	motor 1. Sending a value between 128-255 will command motor 2. 
	Sending a value of zero will shut down both motors" 
	It does not say anything about sending a motor backwards. 
	"""
	# stop condition
	if not(self.linear_vel or self.angular_vel):
	    self.left_vel = 0 
	    self.right_vel = 0
	else: 	
	    # left motor is motor1
	    self.left_vel = (self.linear_vel - self.angular_vel) / 2 + 1
	    self.right_vel = (self.linear_vel + self.angular_vel) / 2 + 128 	
	
    	towrite = bytes([self.left_vel, self.right_vel])
	self.port.write(towrite)

	print("Commanded velocities")
	print("Left: {}".format(self.left_vel))
	print("Right: {}".format(self.right_vel))
	
    def send_linear_vel(self, vel):
        """
        Takes a velocity as a signed integer between -127 and 127.
        Negative values are backwards, zero is stop, and positive values
        are forward.
        """
        # the type checking is somewhat rudimentary.
        try:
            vel = int(vel)
            assert(-128 < vel and 128 > vel)
        except TypeError, AssertionError:
            print 'That is not a valid input'
	
	# all that's needed is to command the velocity.
	self.linear_vel = vel
	
        self.write()

    def send_angular_vel(self, vel):
        """
        takes an angular velocity as a signed integer in the range (-127, 127).
        Negative results are right, zero is stop and positive values are
        left turns.
        """
        # the type checking is somewhat rudimentary.
        try:
            vel = int(vel)
            assert(-128 < vel and 128 > vel)
        except TypeError, AssertionError:
            print "That is not a valid input"
	
	# all that's needed is to command the velocity
	self.angular_vel = vel

        self.write()

    def stop(self):
        self.send_linear_vel(0)
        self.send_angular_vel(0)

if __name__ == "__main__": 
    controller = writer()
    controller.send_linear_vel(10)
    time.sleep(5)
    controller.stop()
