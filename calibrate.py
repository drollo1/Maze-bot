#!/usr/bin/python3
"""calibrate.py has some simple calibration functions to correct the movement of our gopigo.
"""

import scipy.optimize
import numpy as np
import math
import time
import movement

# Servo angles in degrees
SERVO_LEFT = 135
SERVO_FORWARD = 90
SERVO_RIGHT = 45

# Angle between the two measurements used when measuring a wall. The larger this is the smaller
# the error will be, (to a point) but the less likely the measurement will hit the correct wall.
ALIGN_DELTA = 15

# If you execute this library as main then it will try lining the robot up with a wall to the right of it,
# then drive forward and measure the divergence. It's up to you to interpret the change in distance to the
# wall. 
def test():
  while True:
    error = deg(alignToWall(2))
    movement.setServo(0, blocking=True)
    dist = movement.getDistance()
    print("Error: {}   Dist: {}".format(error, dist))
    movement.turn(-1*error, blocking=True)
    movement.driveForward(100, blocking=True)
        
# Convert from radians to degrees    
def deg(rad):
    return rad * 180 / math.pi

# Return the center of a circle that best contains the given points. This is useful as the robot should trace
# a circle due to systematic error in its motion. The center is given relative to the first point. If the center
# of the circle is on the positive X of the robot then right wheel is moving farther than the left, and vice-versa.
# The smaller the absolute value of X the larger the error is.
# If the center of the circle is on the positive Y and X is positive then the servo should be rotated counter-clockwise
# relative to the robot. If Y is negative and X is positive then it should turn clockwise. If Y is negative and X
# is positive then clockwise, and if both are negative then counter-clockwise.
#  X-+
# Y+lr
# --rl
def fitCircle(points):
    def residual(center, points):
        return sum([math.sqrt((center[0]-x[0])**2 + (center[1]-x[1])**2) for x in points])
    res = scipy.optimize.minimize(residual, [0,0], args=(points,))
    center = res['x']
    sumc = sum(center)
    print(center[0]-points[0][0], center[1]-points[0][1])
    return res['x']

# Turn the robot to be parallel to the cell with the given wall
# For "left" or "right" this means being parallel to the wall. For "forward" this means being perpendicular
# Returns an angle in radians representing the error
def alignToWall(wall):
    if wall == 0 or wall == "left":
        servo_initial = SERVO_LEFT - ALIGN_DELTA / 2
        servo_final = SERVO_LEFT + ALIGN_DELTA / 2
    elif wall == 1 or wall == "forward":
        servo_initial = SERVO_FORWARD - ALIGN_DELTA / 2
        servo_final = SERVO_FORWARD + ALIGN_DELTA / 2
    else:
        servo_initial = SERVO_RIGHT - ALIGN_DELTA / 2
        servo_final = SERVO_RIGHT + ALIGN_DELTA / 2
    movement.setServo(servo_initial)
    time.sleep(0.5)
    distance_first = movement.getDistance()
    movement.setServo(servo_final)
    time.sleep(0.5)
    distance_second = movement.getDistance()
    if wall == 0 or wall == "left":
        b = distance_first
        c = distance_second
        A = ALIGN_DELTA/180*math.pi
        a = math.sqrt(b**2 + c**2 - 2*b*c*math.cos(A))
        if b > c:
            B = math.asin((math.sin(A)*b)/a)
            C = (math.pi-B)-A
            theta = -1*(B - math.pi/2)+A+((servo_initial-SERVO_FORWARD)*math.pi/180)
        else:
            B = math.asin((math.sin(A)*b)/a)
            C = (math.pi-B)-A
            theta = (B - math.pi/2)+A+((servo_initial-SERVO_FORWARD)*math.pi/180)
        return math.pi/2 - theta
    if wall == 1 or wall == "forward":
        b = distance_second
        c = distance_first
        A = ALIGN_DELTA/180*math.pi
        a = math.sqrt(b**2 + c**2 - 2*b*c*math.cos(A))
        if b > c:
            B = math.asin((math.sin(A)*b)/a)
            C = (math.pi-B)-A
            theta = math.pi/2-(B-((servo_initial-SERVO_FORWARD)*math.pi/180))
        else:
            B = math.asin((math.sin(A)*b)/a)
            C = (math.pi-B)-A
            theta = (B-((servo_initial-SERVO_FORWARD)*math.pi/180))-math.pi/2
        return theta
    if wall == 2 or wall == "right":
        b = distance_first
        c = distance_second
        A = ALIGN_DELTA/180*math.pi
        a = math.sqrt(b**2 + c**2 - 2*b*c*math.cos(A))
        if b > c:
            B = math.asin((math.sin(A)*b)/a)
            C = (math.pi-B)-A
            theta = -1*(B - math.pi/2)+A+((servo_initial-SERVO_FORWARD)*math.pi/180)
        else:
            B = math.asin((math.sin(A)*b)/a)
            C = (math.pi-B)-A
            theta = (B - math.pi/2)+A+((servo_initial-SERVO_FORWARD)*math.pi/180)
        return theta*-1-math.pi/2
    return 0

if __name__ == "__main__":
    test()
