"""
writer.py

Creates a writer class, that handles sending commands to a Sabertooth
motor controller with a few commands.

Evan Greene
Created 2017-03-17
Updated 2019-06-09
"""
import serial, time

ENABLE_LOGGING = True

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
        if ENABLE_LOGGING:
            print "Opened Port"

        time.sleep(2)

        # doing it once doesn't work. Let's try a bunch times.
        self.port.write([0xAA])

        time.sleep(0.1)
        if ENABLE_LOGGING:
            print "Wrote first byte"

        # initialize values for each of the parameters.

        # start by commanding zero values for each motor
        """
        Address byte will always be 128, unless addressing multiple
        controllers using the same line. Would use a static var, but there's
        no such thing in Python.
        """
        self.address = [128]

        """
        The list of potential commands is
        0   -   Drive motor 1 forwards
        1   -   Drive motor 1 backwards
        2   -   Set a minimum voltage for the battery
        3   -   set a maximum voltage for the battery
        4   -   Drive motor 2 forwards
        5   -   Drive motor 2 backwards
        6   -   Drive motor 1 (7 bit)
                    1 for full reverse, 64 for stop, 127 for full forwards
        7   -   Drive motor 2 (7 bit)
        8   -   Drive forwards mixed mode
        9   -   Drive backwards mixed mode
        10  -   Turn right mixed mode
        11  -   Turn left mixed mode
        12  -   Drive forwards/backwards mixed mode
                    1 for full reverse, 64 for stop, 127 for full forwards
        13  -   Turn 7 bit
                    1 for full left, 64 for dead straight, and 127 for full right
        """
        # tell the motor to drive forwards at zero speed
        self.command = [8]
        # We want to tell the motor not to move.
        self.data = [0]
        # no need to calculate checksum, it's done automatically in the write
        # command
        self.checksum = [0]

        self.write()

        return

    def test(self):
        """
        tests the functionality of the class
        """

        self.send_motor1_vel(64)

        time.sleep(2)

        self.stop_motor1()
        self.send_motor2_vel(64)

        time.sleep(2)

        self.stop_motor2()

        self.send_linear_vel(20)

        time.sleep(2)

        self.stop()

        self.send_angular_vel(20)

        self.stop()
        return

    def write(self):
        """ Computes the checksum then writes the
        current address, command, data, and checksum bytes to the port. """

        self.compute_checksum()

        self.port.write(self.address)
        self.port.write(self.command)
        self.port.write(self.data)
        self.port.write(self.checksum)

        if ENABLE_LOGGING:
            print 'wrote a packet: '
            print 'address = {}'.format(self.address[0])
            print 'command = {}'.format(self.command[0])
            print 'data = {}'.format(self.data[0])
            print 'checksum = {}\n'.format(self.checksum[0])

        return

    def compute_checksum(self):
        self.checksum = [(self.address[0] + self.command[0] +  \
            self.data[0]) & 0b01111111 ]


        # if ENABLE_LOGGING:
        #     print "The checksum is {}".format(self.checksum[0])
        return

    def send_motor1_vel(self, vel):
        """
        Sends a velocity to motor1. Takes an integer between -127 (full reverse)
        and 127 (full forward).
        """
        # the type checking is somewhat rudimentary.
        try:
            vel = int(vel)
            assert(-128 < vel and 128 > vel)
        except TypeError, AssertionError:
            print 'That is not a valid input'

        if vel >= 0:
            self.command[0] = 0
            self.data[0] = vel
        elif vel < 0:
            sel.command[0] = 1
            self.data[0] = abs(vel)

        self.write()

        return

    def stop_motor1(self):
        """
        sends a velocity of zero to motor1
        """

        self.send_motor1_vel(0)

        return

    def send_motor2_vel(self, vel):
        """
        Sends a velocity to motor1. Takes an integer between -127 (full reverse)
        and 127 (full forward).
        """
        # rudimentary type checking
        try:
            vel = int(vel)
            assert(-128 < vel and 128 > vel)
        except TypeError, AssertionError:
            print 'That is not a valid input'

        if vel >= 0:
            self.command[0] = 4
            self.data[0] = vel
        elif vel < 0:
            self.command[0] = 5
            self.data[0] = abs(vel)

        self.write()
        return

    def stop_motor2(self):
        """
        sends a velocity of zero to motor2
        """

        self.send_motor2_vel(0)

        return

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
        Negative results are left, zero is stop and positive values are
        right turns.
        """
        # the type checking is somewhat rudimentary.
        try:
            vel = int(vel)
            assert(-128 < vel and 128 > vel)
        except TypeError, AssertionError:
            print "That is not a valid input"

        if vel == 0:
            self.command[0] = 10
            self.data[0] = 0

        elif vel > 0:
            self.command[0] = 10
            self.data[0] = vel

        elif vel < 0:
            self.command[0] = 11
            self.data[0] = abs(vel)

        self.write()

    def stop(self):
        self.send_linear_vel(0)
        self.send_angular_vel(0)

    # this seems to be causing some bugs, so it's disabled.
    # def __del__(self):
    #     self.stop() # Not necessary because of the timeout.
    #     # self.port.close()

class simplified_serial_writer(object):
    """
    A class for sending commands in the simplified serial mode
    """

    def __init__(self):

        # Open a port
        try: # should work on linux
            # self.port = serial.Serial('/dev/cu.usbserial')
            self.port = serial.Serial('/dev/ttyUSB0', baudrate = 9600)
        except serial.serialutil.SerialException: # should work on mac
            self.port = serial.Serial('/dev/cu.usbserial-A6033KY3',
                baudrate = 9600)
        if ENABLE_LOGGING:
            print "Opened Port"



    def send1(self, vel):
        """
        sends the input vel to motor 1. Input can be between -63 for full
        reverse and 63 for full forwards. Zero for stop.
        """
        # some rudimentary type checking
        try:
            vel = int(vel)
            assert(vel > -64 and vel < 64)
        except TypeError, AssertionError:
            print "That is not a valid input"

        vel += 64

        self.write([vel | 0b10000000])

        return


    def stop1(self):
        """
        sends a stop command to motor 1
        """
        self.send1(0)

        return

    def send2(self, vel):
        """
        Sends the input vel to motor 2. Input can be between -63 for full
        reverse and 63 for full forwards. Zero for stop.
        """
        # some rudimentary type checking
        try:
            vel = int(vel)
            assert(vel > -64 and vel < 64)
        except TypeError, AssertionError:
            print "That is not a valid input"

        vel += 64

        self.write([vel & 0b01111111])

        return

    def stop2(self):
        """
        sends a stop command to motor 2
        """
        self.send2(0)

        return

    def write(self, towrite):
        """
        writes the command to the port
        """
        self.port.write(towrite)
        if ENABLE_LOGGING:
            print("Wrote a byte: {}".format(int(towrite)))

        return

    def test(self):
        """
        tests the functions of the simplified_serial_writer class
        """
        time.sleep(1)

        self.send1(20)

        time.sleep(2)

        self.stop1()
        self.send2(20)

        time.sleep(2)

        self.stop2()

    def advance(self):
        """
        Advances the robot slowly for a few seconds
        """
        vel = 0 # start off at zero velocity
        rampTime = 0.5 # time taken to ramp up and down.
        maxVel = 20 # the highest velocity reached

        # ramp up to our maximum velocity.
        for i in xrange(100):
            vel = int(maxvel * float(i) / 100)
            self.send1(vel)
            self.send2(vel)
            time.sleep(rampTime / 100)

        # keep things level for a bit.
        for i in xrange(100):
            self.send1(maxVel)
            self.send2(maxvel)
            time.sleep(rampTime / 100)

        # now ramp down.
        for i in xrange(100):
            vel = int(maxvel - maxvel * float(i) / 100)
            self.send1(vel)
            self.send2(vel)
            time.sleep(ramptime / 100)

        # now make sure everything is stopped
        self.stop1()
        self.stop2()

    def test(self):
        """
        Has the robot slowly spin one wheel, then the other
        """
        vel = 10

        self.send1(vel)
        if ENABLE_LOGGING:
            print("spinning right wheel")

        time.sleep(1.5)

        self.stop1()
        if ENABLE_LOGGING:
            print("stopped right wheel")

        self.send2(vel)
        if ENABLE_LOGGING:
            print("spinning left wheel")

        time.sleep(1.5)
        self.stop2()
        if ENABLE_LOGGING:
            print("stopped left wheel")


if __name__ == '__main__':
    # controller = writer()
    controller = writer()
    controller.test()


    # controller.stop()
