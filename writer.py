"""
writer.py

Creates a writer class, that handles sending commands to a Sabertooth
motor controller with a few commands.

Evan Greene
Created 2017-03-17
Updated 2019-06-09
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

        # initialize values for address, command, and data, and the checksum
        self.address = bytearray([128])
        self.command = bytearray([14])
        self.data = bytearray([30])
        self.checksum = bytearray([0])

        # set up a timeout, where the motor will time out after not
        # recieving a command for five seconds.
        self.write()




    def write(self):
        """ Computes the checksum then writes the
        current address, command, data, and checksum bytes to the port. """

        self.compute_checksum()

        towrite = bytearray()
        for elt in [self.address, self.command, self.data, \
            self.checksum]:

            towrite.extend(elt)

        self.port.write(towrite)
        print 'wrote a packet: '
    	print 'address = {}'.format(self.address[0])
    	print 'command = {}'.format(self.command[0])
    	print 'data = {}'.format(self.data[0])
    	print 'checksum = {}\n'.format(self.checksum[0])

    def compute_checksum(self):
        self.checksum = bytearray([(self.address[0] + self.command[0] +  \
            self.data[0]) & 0b01111111 ])

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
        if vel == 0: # stop case
            self.command[0] = 8
            self.data[0] = 0

        elif vel > 0: # forwards case
            self.command[0] = 8
            self.data[0] = vel

        elif vel < 0: # backwards case
            self.command[0] = 9
            self.data[0] = abs(vel)

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

        if vel == 0:
            self.command[0] = 11
            self.data[0] = 0

        elif vel > 0:
            self.command[0] = 11
            self.data[0] = vel

        elif vel < 0:
            self.command[0] = 10
            self.data[0] = abs(vel)

        self.write()

    def stop(self):
        self.send_linear_vel(0)
        self.send_angular_vel(0)

    # this seems to be causing some bugs, so it's disabled.
    # def __del__(self):
    #     self.stop() # Not necessary because of the timeout.
    #     # self.port.close()

if __name__ == '__main__':
    controller = writer()

    controller.send_linear_vel(60)
    controller.send_angular_vel(0)


    time.sleep(5)

    controller.stop()
