#!/usr/bin/python3
import movement

while True:
    angle = int(input(":"))
    movement.setServo(angle)
