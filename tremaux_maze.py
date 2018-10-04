#!/usr/bin/python3

import time
import movement
import calibrate
import random

# Preset servo directions in degrees, following right hand rule with thumb facing up.
# 0 is directly right, orthogonal to the robots forward movement
SERVO_FORWARD = 90
SERVO_LEFT = 145
SERVO_RIGHT = 35

GRID_MOVE_DISTANCE = 280 # length of each grid cell of the maze, in mm
GRID_SCAN_SIDE_DISTANCE = 300 # How close an obstacle has to be in order to prevent turning in that direction
GRID_SCAN_FORWARD_DISTANCE = 400 # How close an obstacle has to be in order to prevent going forward

# Check the next cell to see which sides of it are open.
# If a side has a wall, then measure our heading relative to that wall. Average the error.
# returns:
#   options
#     A list of which directions we can move in the next cell. Either "left", "forward", or "right".
#   avgError
#     The amount we will have to turn to best align ourselves to the walls in the next cell. If there
#     are no walls on the next cell this will be 0. Otherwise a positive error means the robot should
#     turn counterclockwise avgError degrees. This does not center us in the cell, it only puts us
#     parallel to the walls.
def print_maze(maze):
    minx = 0
    miny = 0
    maxx = 0
    maxy = 0
    for i in maze.keys():
        if i[0] > maxx:
            maxx = i[0]
        if i[0] < minx:
            minx = i[0]
        if i[1] > maxy:
            maxy = i[1]
        if i[1] < maxy:
            maxy = i[1] 
    for y in range(miny, maxy+1):
        for i in range(2):
            for x in range(minx, maxx+1):
                if not (x,y,"left") in maze:
                    continue
                if maze[(x,y,"left")]["wall"]:
                    print("|    ", end="")
                else:
                    print("     ", end="")
            print()
        for x in range(minx, maxx+1):
            if not (x,y,"left") in maze:
                continue
            if maze[(x,y,"left")]["wall"]:
                print("| {}  ".format(maze[(x,y,"left")]["visits"]), end="")
            else:
                print("  {}  ".format(maze[(x,y,"left")]["visits"]), end="")
        
        for i in range(2):
            for x in range(minx, maxx+1):
                if not (x,y,"left") in maze:
                    continue
                if maze[(x,y,"left")]["wall"]:
                    print("|    ", end="")
                else:
                    print("     ", end="")
            print()

        for x in range(minx, maxx+1):
            if not (x,y,"left") in maze:
                continue
            if not (x,y,"bottom") in maze:
                continue
            if maze[(x,y,"left")]["wall"] and maze[(x,y,"bottom")]["wall"]:
                print("+----", end="")
            elif maze[(x,y,"bottom")]["wall"]:
                print("-----", end="")
            else:
                print("     ", end="")
        print()

def checkWalls(location, maze):
    options = []
    avgError = 0
    errorweight = 0

    def set_wall(maze, location, offset, value):
        angle = location[2] + offset
        angle %= 4
        if angle == 0:
            maze[(location[0], location[1]+1, "bottom")]["wall"] = value
            return maze[(location[0], location[1]+1, "bottom")]["visits"]
        elif angle == 1:
            maze[(location[0], location[1], "left")]["wall"] = value
            return maze[(location[0], location[1], "left")]["visits"]
        elif angle == 2:
            maze[(location[0], location[1], "bottom")]["wall"] = value
            return maze[(location[0], location[1], "bottom")]["visits"]
        elif angle == 3:
            maze[(location[0]+1, location[1], "left")]["wall"] = value
            return maze[(location[0]+1, location[1], "left")]["visits"]
        print("Got invalid angle {}".format(angle))
        return False

    # Check the right wall
    movement.setServo(SERVO_RIGHT, blocking=True)
    distance = movement.getDistance()
    if distance > GRID_SCAN_SIDE_DISTANCE:
        if set_wall(maze, location, -1, True) < 2:
            options.append("right")
    else:
        avgError += calibrate.alignToWall("right")
        errorweight += 1
        avgError += calibrate.alignToWall("right")
        errorweight += 1
        set_wall(maze, location, -1, False)

    # Check the front wall
    movement.setServo(SERVO_FORWARD, blocking=True)
    distance = movement.getDistance()
    if distance > GRID_SCAN_FORWARD_DISTANCE:
        if set_wall(maze, location, 0, True) < 2:
            options.append("forward")
    else:
        avgError += calibrate.alignToWall("forward")
        errorweight += 1
        avgError += calibrate.alignToWall("forward")
        errorweight += 1
        set_wall(maze, location, 0, False)


    # Check the right wall
    movement.setServo(SERVO_LEFT, blocking=True)
    distance = movement.getDistance()
    if distance > GRID_SCAN_SIDE_DISTANCE:
        if set_wall(maze, location, 1, False) < 2:
            options.append("left")
    else:
        avgError += calibrate.alignToWall("left")
        errorweight += 1
        avgError += calibrate.alignToWall("left")
        errorweight += 1
        set_wall(maze, location, 1, True)

    # If we saw any walls (meaning that direction isn't an option to move) then
    # divide the avgError by the number of walls we saw.
    if errorweight:
        avgError /= errorweight
    avgError = calibrate.deg(avgError) # convert avgError from radians to degrees
    return options, avgError

def main():
    maze = {
      (0,0,"bottom"): {"wall": True, "visits": 1},
      (0,0,"left"): {"wall": True, "visits": 1},
      (0,1,"bottom"): {"wall": False, "visits": 0},
      (0,1,"left"): {"wall": None, "visits": 0},
      (1,0,"bottom"): {"wall": None, "visits": 0},
      (1,0,"left"): {"wall": True, "visits": 0},
    }

    location = [0,0,0]
    while True:
        print(maze)
        # Look into the next cell to figure out what moves will be available, and how 
        # well we are aligned to its walls
        options, avgError = checkWalls(location, maze)

        # If we are not parallel to the next cell then adjust our heading before we go forward
        if avgError:
            print("Measured {} degree error.".format(avgError))
            if abs(avgError) > 15: # Really big errors were probably either found too late, or measured wrong
                print("Error is strangely large...") # If you see this then something likely went wrong.
            else:
                time.sleep(0.5)
                # Rotate in the opposite direction of the measured error
                movement.turn(-1*avgError, blocking=True)
                time.sleep(0.5)

        # If any walls are open in the next cell then pick which path we are taking
        if options:
            choice = options[0]
        else: # If no walls are open then we will turn around
            choice = "turn_around"

        # If we have more than one option then we had a decision to make
        decision_point = len(options) > 1

        if decision_point:
            print("Decision point found. Options: {}".format(", ".join(options)))
            print("DECISION: Let's go {}.".format(choice))

        time.sleep(0.5)
        # Point the servo forward to detect a wall in front of us in case we overshoot while moving
        movement.setServo(SERVO_FORWARD, blocking=True)
        # If you use driveForward in blocking mode then it will check the distance sensor while moving
        # and abort early if it sees something too close.
        movement.driveForward(GRID_MOVE_DISTANCE, blocking=True)  
        if location[2] == 0:
            location[1] += 1
        elif location[2] == 1:
            location[0] -= 1
        elif location[2] == 2:
            location[1] -= 1
        elif location[2] == 3:
            location[0] += 1
        else:
            sys.exit("Illegal orientation {}".format(location[2]))
        if not((location[0], location[1], "bottom")) in maze:
            maze[(location[0], location[1], "bottom")] = {"wall": None, "visits": 0}
        if not((location[0], location[1], "left")) in maze:
            maze[(location[0], location[1], "left")] = {"wall": None, "visits": 0}
        if not((location[0], location[1]+1, "bottom")) in maze:
            maze[(location[0], location[1]+1, "bottom")] = {"wall": None, "visits": 0}
        if not((location[0]+1, location[1], "left")) in maze:
            maze[(location[0]+1, location[1], "left")] = {"wall": None, "visits": 0}

        maze[(location[0], location[1], "bottom")]["visits"] += 1
        maze[(location[0], location[1], "left")]["visits"] += 1
        time.sleep(0.5)

        # Now that we are in the next cell we can turn wherever we decided.
        if choice == "turn_around":
            movement.turn(180, blocking=True)
            location[2] += 2
            location[2] %= 4
        elif choice == "left":
            movement.turn(90)
            location[2] -= 1
            location[2] %= 4
        elif choice == "right":
            movement.turn(-90)
            location[2] -= 1
            location[2] %= 4
        elif choice == "forward":
            pass
        else:
            # This really shouldn't happen.
            sys.exit("Unknown choice: {}".format(choice))

        time.sleep(0.5)            
main()
