#!/usr/bin/python3

import time
import movement

def main():
    while input("Press e to exit:") != "e":
        movement.driveForward(1000, blocking=True)
        time.sleep(0.5)
        movement.turn(90, blocking=True)
        time.sleep(0.5)
        movement.driveForward(1000, blocking=True)
        time.sleep(0.5)
        movement.turn(90, blocking=True)
        time.sleep(0.5)
        movement.driveForward(1000, blocking=True)
        time.sleep(0.5)
        movement.turn(90, blocking=True)
        time.sleep(0.5)
        movement.driveForward(1000, blocking=True)
        time.sleep(0.5)
        movement.turn(90, blocking=True)
main()
