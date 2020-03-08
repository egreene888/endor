"""
writer_serial.py

Written by Evan Greene
2019-12-02

For use commanding a Sabertooth 2x5 in simplified serial mode.

"""
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

        towrite = bytes([self.left_vel, self.right_vel])
        self.port.write(towrite)

        print("Commanded velocities")
        print("Left: {}".format(self.left_vel))
        print("Right: {}".format(self.right_vel))

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
        except TypeError, AssertionError:
            print 'That is not a valid input'

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
        except TypeError, AssertionError:
            print "That is not a valid input"

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

        self.port.write(bytes([0]))


if __name__ == "__main__":
    controller = writer()
    controller.send_linear_vel(10)
    time.sleep(5)
    controller.stop()
