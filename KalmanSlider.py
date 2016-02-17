#!/usr/bin/python

# Code adapted from Georgia Tech CS8803, final quiz of lecture #2 by Jon Keller as follows:
# 1. Completed the work assigned in the quiz
# 2. Modified the solution to drive a motor and camera from a Raspberry Pi
###########################################################################################

# Motor code from https://learn.adafruit.com/adafruit-raspberry-pi-lesson-9-controlling-a-dc-motor/software
import RPi.GPIO as GPIO
GPIO.setmode(GPIO.BCM)

in1_pin = 4
in2_pin = 17

GPIO.setup(in1_pin, GPIO.OUT)
GPIO.setup(in2_pin, GPIO.OUT)
GPIO.setup(18, GPIO.OUT)
motor = GPIO.PWM(18, 100)

#def set(property, value):
#    try:
#        f = open("/sys/class/rpi-pwm/pwm0/" + property, 'w')
#        f.write(value)
#        f.close()
#    except:
#        print("Error writing to: " + property + " value: " + value)
#
#set("delayed", "0")
#set("mode", "pwm")
#set("frequency", "500")
#set("active", "1")

def clockwise():
    GPIO.output(in1_pin, True)
    GPIO.output(in2_pin, False)

def counter_clockwise():
    GPIO.output(in1_pin, False)
    GPIO.output(in2_pin, True)

# Set up the bumper switches:
left_bumper_pin = 24
right_bumper_pin = 25
GPIO.setup(right_bumper_pin, GPIO.IN)
GPIO.setup(left_bumper_pin, GPIO.IN)

# Raspberry Pi camera imports

import io
import time
import picamera
from PIL import Image


# Here begins the CS8803 code:

# Write a function 'filter' that implements a multi-
# dimensional Kalman Filter for the example given

from math import *

class matrix:

    # implements basic operations of a matrix class

    def __init__(self, value):
        self.value = value
        self.dimx = len(value)
        self.dimy = len(value[0])
        if value == [[]]:
            self.dimx = 0

    def zero(self, dimx, dimy):
        # check if valid dimensions
        if dimx < 1 or dimy < 1:
            raise ValueError, "Invalid size of matrix"
        else:
            self.dimx = dimx
            self.dimy = dimy
            self.value = [[0 for row in range(dimy)] for col in range(dimx)]

    def identity(self, dim):
        # check if valid dimension
        if dim < 1:
            raise ValueError, "Invalid size of matrix"
        else:
            self.dimx = dim
            self.dimy = dim
            self.value = [[0 for row in range(dim)] for col in range(dim)]
            for i in range(dim):
                self.value[i][i] = 1

    def show(self):
        for i in range(self.dimx):
            print self.value[i]
        print ' '

    def __add__(self, other):
        # check if correct dimensions
        if self.dimx != other.dimx or self.dimy != other.dimy:
            raise ValueError, "Matrices must be of equal dimensions to add"
        else:
            # add if correct dimensions
            res = matrix([[]])
            res.zero(self.dimx, self.dimy)
            for i in range(self.dimx):
                for j in range(self.dimy):
                    res.value[i][j] = self.value[i][j] + other.value[i][j]
            return res

    def __sub__(self, other):
        # check if correct dimensions
        if self.dimx != other.dimx or self.dimy != other.dimy:
            raise ValueError, "Matrices must be of equal dimensions to subtract"
        else:
            # subtract if correct dimensions
            res = matrix([[]])
            res.zero(self.dimx, self.dimy)
            for i in range(self.dimx):
                for j in range(self.dimy):
                    res.value[i][j] = self.value[i][j] - other.value[i][j]
            return res

    def __mul__(self, other):
        # check if correct dimensions
        if self.dimy != other.dimx:
            raise ValueError, "Matrices must be m*n and n*p to multiply"
        else:
            # subtract if correct dimensions
            res = matrix([[]])
            res.zero(self.dimx, other.dimy)
            for i in range(self.dimx):
                for j in range(other.dimy):
                    for k in range(self.dimy):
                        res.value[i][j] += self.value[i][k] * other.value[k][j]
            return res

    def transpose(self):
        # compute transpose
        res = matrix([[]])
        res.zero(self.dimy, self.dimx)
        for i in range(self.dimx):
            for j in range(self.dimy):
                res.value[j][i] = self.value[i][j]
        return res

    # Thanks to Ernesto P. Adorio for use of Cholesky and CholeskyInverse functions

    def Cholesky(self, ztol=1.0e-5):
        # Computes the upper triangular Cholesky factorization of
        # a positive definite matrix.
        res = matrix([[]])
        res.zero(self.dimx, self.dimx)

        for i in range(self.dimx):
            S = sum([(res.value[k][i])**2 for k in range(i)])
            d = self.value[i][i] - S
            if abs(d) < ztol:
                res.value[i][i] = 0.0
            else:
                if d < 0.0:
                    raise ValueError, "Matrix not positive-definite"
                res.value[i][i] = sqrt(d)
            for j in range(i+1, self.dimx):
                S = sum([res.value[k][i] * res.value[k][j] for k in range(self.dimx)])
                if abs(S) < ztol:
                    S = 0.0
                res.value[i][j] = (self.value[i][j] - S)/res.value[i][i]
        return res

    def CholeskyInverse(self):
        # Computes inverse of matrix given its Cholesky upper Triangular
        # decomposition of matrix.
        res = matrix([[]])
        res.zero(self.dimx, self.dimx)

        # Backward step for inverse.
        for j in reversed(range(self.dimx)):
            tjj = self.value[j][j]
            S = sum([self.value[j][k]*res.value[j][k] for k in range(j+1, self.dimx)])
            res.value[j][j] = 1.0/tjj**2 - S/tjj
            for i in reversed(range(j)):
                res.value[j][i] = res.value[i][j] = -sum([self.value[i][k]*res.value[k][j] for k in range(i+1, self.dimx)])/self.value[i][i]
        return res

    def inverse(self):
        aux = self.Cholesky()
        res = aux.CholeskyInverse()
        return res

    def __repr__(self):
        return repr(self.value)


########################################

# Implement the filter function below

def kalman_filter(x, P):
    for n in range(numMotions):

        # measurement update
        measurement = doHardwareSense()
        z = matrix([[measurement]])
        y = z - H*x
        S = H * P * Htranspose + R
        Sinverse = S.inverse()
        K = P * Htranspose * Sinverse
        x = x + K*y
        P = (I - K*H) * P

        # prediction
        doHardwareMotion(motionDistance)
        x = F * x + u
        P = F * P * Ftranspose

        print "After sense/move #" + str(n) + ":"
        print "x = " + str(x)
        print "P = "
        print P
    return x,P

# Take a picture with the Raspberry Pi camera, and return it as a PIL image
def takeAPicture():
    #print "Going to take a picture"
    # Create the in-memory stream
    stream = io.BytesIO()
    with picamera.PiCamera() as camera:
        camera.start_preview()
        time.sleep(2)
        camera.capture(stream, format='jpeg')
    # "Rewind" the stream to the beginning so we can read its content
    stream.seek(0)
    image = Image.open(stream)
    #print "Took a picture"
    return image

# In my setup, I have a camera pointing to a 1-dimensional slider. On the slider is a red piece of plastic.
# The slider is aluminum and the wall is gray.  The slider can carry the red target horizontally almost
# across the entire field of view of the camera.
# So this method takes a picture and returns a number indicating how far from the left edge the red target is.
def doHardwareSense():
    print "\n\nBeginning sense..."
    # Take a picture
    image = takeAPicture()
    image.save("image.png")

    width = image.size[0]

    # My image is 720x480, the target goes across the middle, and there are some
    # red wires at the bottom of the image that I want to crop out
    image = image.crop((0, 120, width, 300))
    image.save("croppedImage.png")
    data = image.load()

    redPixelCount = 0
    redPixelTotalX = 0

    # Isolate red pixels

    for y in xrange(image.size[1]):
        for x in xrange(width):
            pixel = data[x, y]
            if (pixel[0] > 2*pixel[1] and pixel[0] > 2*pixel[2]):
                redPixelCount += 1
                redPixelTotalX += x
                data[x, y] = (255, 0, 0, 255)
            else:
                data[x, y] = (0, 0, 0, 255)

    # Find mean of red pixels
    mean = redPixelTotalX / redPixelCount

    print "Sensed value: " + str(mean)
    image.save("redPixels.png")
    return mean

# This method moves the slider to the right by the specified distance.
# The units of this distance depend on battery strength, etc. so don't construe this as mm, inches, etc.
def doHardwareMotion(distance):
    # Move the slider by distance (at hard-coded speed)
    print "Going to move: " + str(distance)
    travelled = 0
    clockwise()
    startMotor()
    while travelled < distance and bumperSwitchOpen(right_bumper_pin):
        travelled += 1
        time.sleep(motor_unit_distance_time)
    stopMotor()
    print "Move complete"

# Move the slider all the way to the left.
def resetSlider():
    print "Going to reset the slider"
    # Move the motor to the left, until the left-end bumper switch becomes closed.
    counter_clockwise()
    startMotor()
    while bumperSwitchOpen(left_bumper_pin):
        time.sleep(motor_unit_distance_time)
    stopMotor()
    print "Slider reset complete"

def startMotor():
    #set("duty", speed)
    motor.start(speed)

def stopMotor():
    #set("duty", "0")
    motor.stop()

def bumperSwitchOpen(bumperSwitchPin):
    #print "Checking whether bumper switch is OPEN..."
    isOpen = not not GPIO.input(bumperSwitchPin)
    #print str(isOpen)
    return isOpen

start_pos = 144
motionDistance = 20
numMotions = 10
speed = 99 # 0...99
motor_unit_distance_time = .058 # seconds for the motor to go a distance of "1"
resetSlider()

############################################
### use the code below to test your filter!
############################################

#measurements = [1, 2, 3]

x = matrix([[start_pos], [0.]]) # initial state (location and velocity)
P = matrix([[1000., 0.], [0., 1000.]]) # initial uncertainty
u = matrix([[0.], [0.]]) # external motion
F = matrix([[1., 1.], [0, 1.]]) # next state function
H = matrix([[1., 0.]]) # measurement function
R = matrix([[1.]]) # measurement uncertainty
I = matrix([[1., 0.], [0., 1.]]) # identity matrix

Htranspose = H.transpose()
Ftranspose = F.transpose()

kalman_filter(x, P)
