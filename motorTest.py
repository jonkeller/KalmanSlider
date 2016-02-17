#!/usr/bin/python

import io
import time
import RPi.GPIO as GPIO
GPIO.setmode(GPIO.BCM)

pwm_pin = 18
in1_pin = 4
in2_pin = 17

GPIO.setup(in1_pin, GPIO.OUT)
GPIO.setup(in2_pin, GPIO.OUT)
GPIO.setup(pwm_pin, GPIO.OUT)
motor = GPIO.PWM(pwm_pin, 100)

def clockwise():
    GPIO.output(in1_pin, True)
    GPIO.output(in2_pin, False)

def counter_clockwise():
    GPIO.output(in1_pin, False)
    GPIO.output(in2_pin, True)

# Set up the bumper switches:
right_bumper_pin = 25
left_bumper_pin = 24
GPIO.setup(right_bumper_pin, GPIO.IN)
GPIO.setup(left_bumper_pin, GPIO.IN)

# This method moves the slider to the right by the specified distance.
# The units of this distance depend on battery strength, etc. so don't construe this as mm, inches, etc.
def doHardwareMotion(distance):
    # Move the slider by distance (at hard-coded speed)
    print "Going to move: " + str(distance)
    travelled = 0
    clockwise()
    startMotor()
    while travelled < distance and bumperSwitchOpen(right_bumper_pin):
        travelled += distance
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
        pass
    stopMotor()
    print "Slider reset complete"

def startMotor():
    #set("duty", speed)
    motor.start(speed)

def stopMotor():
    #set("duty", "0")
    motor.stop()

def bumperSwitchOpen(bumperSwitchPin):
    print "Checking whether bumper switch is OPEN..."
    isOpen = not not GPIO.input(bumperSwitchPin)
    print str(isOpen)
    return isOpen

speed = 99 # 0...99
motor_unit_distance_time = .500 # seconds for the motor to go a distance of "1"
#resetSlider()
print "Left pin, which is #" + str(left_bumper_pin)
print bumperSwitchOpen(left_bumper_pin)
print "Right pin, which is #" + str(right_bumper_pin)
print bumperSwitchOpen(right_bumper_pin)

#counter_clockwise()
#startMotor()
#time.sleep(10)
#stopMotor()

#clockwise()
#startMotor()
#time.sleep(10)
#stopMotor()


GPIO.cleanup()
