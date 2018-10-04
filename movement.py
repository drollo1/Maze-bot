"""movement.py is a simple wrapper for the gopigo3 library, similar to easygopigo3 but
<strike>better</strike> purpose-built for our project. It has hard-coded measurements of
our particular robot.
"""
from di_sensors.distance_sensor import DistanceSensor

import sys
import math
import time
import random
import numpy as np
import scipy.optimize
import gopigo3

gpg = gopigo3.GoPiGo3()

WHEEL_DISTANCE = 109.75 # The distance between the sticking points of the wheels while turning, in mm
MOTOR_SPEED = 100 # Movement speed in mm/s
LEFT_RADIUS = 32.35 # effective wheel radius in mm of the left wheel
RIGHT_RADIUS = 32.35 # effective wheel radius in mm of the right wheel
LEFT_CIRCUMFERENCE = 2*math.pi*LEFT_RADIUS
RIGHT_CIRCUMFERENCE = 2*math.pi*RIGHT_RADIUS
LEFT_SPEED = (MOTOR_SPEED / LEFT_CIRCUMFERENCE) * 360 # Degrees per second
RIGHT_SPEED = (MOTOR_SPEED / RIGHT_CIRCUMFERENCE) * 360 # Degrees per second
BACKLASH = 1 # How much backlash our gearboxes have when reversing direction, in degrees
SERVO_MAX = 2550 # PWM duty cycle to set servo to 180 degrees (fully counter-clockwise)
SERVO_MIN = 720 # PWM duty cycle to set servo to 0 degrees (fully clockwise)
SERVO_RANGE = 180 # How many degrees are between servo_max and servo_min

# Initial values for backlash (False assumes last movement was positive)
left_backlash = False
right_backlash = False

# Initial assumed servo position, in degrees. This is used to estimate travel time for blocking.
servo_pos = 90

# Configure servo out number 1 for PWM
gpg.set_grove_type(gpg.SERVO_1, gpg.GROVE_TYPE.CUSTOM)
gpg.set_grove_mode(gpg.SERVO_1, gpg.GROVE_OUTPUT_PWM)

# Set the motor speeds
gpg.set_motor_limits(gpg.MOTOR_LEFT, dps=LEFT_SPEED)
gpg.set_motor_limits(gpg.MOTOR_RIGHT, dps=RIGHT_SPEED)

# Initialize the distance sensor on I2C number 1, the default
distance_sensor = DistanceSensor()

# Move the servo to the given angle in degrees. angle=0 means it points to the right,
# orthogonal to the forward motion of the robot.
# If blocking is True then wait sufficient time for the servo to move. There is no 
# feedback for this, but if it is free then this should be a safe amount of time.
def setServo(angle, blocking=False):
    global servo_pos
    pulsewidth = int((SERVO_MIN) + ((SERVO_MAX-SERVO_MIN)/SERVO_RANGE)*angle)
    gpg.set_servo(gpg.SERVO_1, pulsewidth)
    if blocking:
        time.sleep(0.5*abs(angle-servo_pos)/180)
    servo_pos = angle

# If you _really_ know what you are doing then you can directly set the PWM duty cycle
# for the servo. This does not have a blocking feature, and does not update the blocking time.
def setServoRaw(pulsewidth):
    gpg.set_servo(gpg.SERVO_1, pulsewidth)

# Read the distance sensor and drop any readings outside one standard deviation. readings=10 is
# pretty safe, but a bit slow.
def getDistance(readings=10):
    data = [distance_sensor.read_range_single() for x in range(readings)]
    data = [x for x in data if abs(x - np.mean(data)) < x*np.std(data)]
    if not(data):
        data.append(distance_sensor.read_range_single())
    avg = np.mean(data)
    return avg

# Blocks until either the wheels both reach their targets (+/- 5 degrees, which is hardcoded in
# the firmware) or the callback returns True. cbargs and cbkwargs are passed through to the callback.
def waitForTarget(leftTarget, rightTarget, callback=None, cbargs=(), cbkwargs={}):
    left = gpg.get_motor_encoder(gpg.MOTOR_LEFT)
    right = gpg.get_motor_encoder(gpg.MOTOR_RIGHT)
    if leftTarget is None:
        leftTarget = left
    if rightTarget is None:
        rightTarget = right
    leftDir = left > leftTarget
    rightDir = right > rightTarget
    leftDone = False
    rightDone = False
    while not (leftDone and rightDone):
        left = gpg.get_motor_encoder(gpg.MOTOR_LEFT)
        right = gpg.get_motor_encoder(gpg.MOTOR_RIGHT)
        if (leftDir and (leftTarget - left >= -5)) or (not(leftDir) and (leftTarget - left <= 5)):
            leftDone = True
        if (rightDir and (rightTarget - right >= -5)) or (not(rightDir) and (rightTarget - right <= 5)):
            rightDone = True
        if callback:
            if callback(*cbargs, **cbkwargs):
                return
        else:
            time.sleep(0.1)
    return 

# Callback meant for waitForTarget that will halt the movement if the distance sensor sees something too close.
# Note that it does not move the servo, so you'll have to do that before moving.
def stopAtObstacle(distance=50):
    dist = getDistance()
    if dist < distance:
        gpg.set_motor_dps(gpg.MOTOR_LEFT + gpg.MOTOR_RIGHT, 0)
        print("Encountered obstacle at {}mm < {}mm".format(dist, distance))
        return True
    return False

# Go forward distance (in mm). If blocking then don't return until we get there, or we get too close to something along the way.
def driveForward(distance, blocking=True):
    global left_backlash
    global right_backlash
    LeftWheelTurnDegrees = distance / LEFT_CIRCUMFERENCE * 360
    RightWheelTurnDegrees = distance / RIGHT_CIRCUMFERENCE * 360
    StartPositionLeft = gpg.get_motor_encoder(gpg.MOTOR_LEFT)
    StartPositionRight = gpg.get_motor_encoder(gpg.MOTOR_RIGHT)
    if left_backlash:
        LeftWheelTurnDegrees += BACKLASH
        left_backlash = False
    if right_backlash:
        RightWheelTurnDegrees += BACKLASH
        right_backlash = False
    gpg.set_motor_position(gpg.MOTOR_LEFT, (StartPositionLeft + LeftWheelTurnDegrees))
    gpg.set_motor_position(gpg.MOTOR_RIGHT, (StartPositionRight + RightWheelTurnDegrees))
    if blocking:
        waitForTarget(StartPositionLeft + LeftWheelTurnDegrees, StartPositionRight + RightWheelTurnDegrees, callback=stopAtObstacle, cbkwargs={"distance": 40})

# Turn counter-clockwise (positive rotation by right-hand rule) angle degrees. If blocking is true then don't return until we get there.
def turn(angle, blocking=True):
    global left_backlash
    global right_backlash
    distance = abs(WHEEL_DISTANCE*math.pi*(angle/360))
    LeftWheelTurnDegrees = distance / LEFT_CIRCUMFERENCE * 360
    RightWheelTurnDegrees = distance / RIGHT_CIRCUMFERENCE * 360
    if angle > 0:
        LeftWheelTurnDegrees *= -1
        if not(left_backlash):
            LeftWheelTurnDegrees -= BACKLASH
            left_backlash = True
        if right_backlash:
            RightWheelTurnDegrees += BACKLASH
            right_backlash = False
    else:
        RightWheelTurnDegrees *= -1
        if not(right_backlash):
            RightWheelTurnDegrees -= BACKLASH
            right_backlash = True
        if left_backlash:
            LeftWheelTurnDegrees += BACKLASH
            left_backlash = False
    StartPositionLeft = gpg.get_motor_encoder(gpg.MOTOR_LEFT)
    StartPositionRight = gpg.get_motor_encoder(gpg.MOTOR_RIGHT)
    gpg.set_motor_position(gpg.MOTOR_LEFT, (StartPositionLeft + LeftWheelTurnDegrees))
    gpg.set_motor_position(gpg.MOTOR_RIGHT, (StartPositionRight + RightWheelTurnDegrees))
    if blocking:
        waitForTarget(StartPositionLeft + LeftWheelTurnDegrees, StartPositionRight + RightWheelTurnDegrees)

def main(state):
    print("Starting movement thread")
    while True:
        time.sleep(0.000001)
