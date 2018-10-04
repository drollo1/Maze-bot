#!/usr/bin/python3
import numpy as np
import serial
import struct
import time
import math

min_distance = 140
angle_offset = -105
reverse = True

START_SENTINEL = 0xFA

def main(state, callback):
    print("Starting lidar thread")
    serial_buffer = bytes()
    with serial.Serial("/dev/ttyS0", 115200, timeout=1) as serial_port:
        while serial_port.isOpen:
            time.sleep(0.000001)
            serial_buffer += serial_port.read()
            readings, serial_buffer = parse(serial_buffer)
            if readings and callback:
                callback(readings)

def send( angle, distance ):
    angle = (-1 if reverse else 1) * (angle + angle_offset)
    angle %= 360
    recv_time = time.time()

    # Mask top two bits of distance (only 14 bits needed)
    data[1] &= 0x3F
    distance = struct.unpack_from('<H', data)[0]
    quality = struct.unpack_from('<H', data, offset=2)[0]

def parse(data):
    # Look for the start of a packet in our buffer
    idx = data.find(START_SENTINEL)

    # If we didn't find the start of a packet then keep waiting
    if idx < 0:
        return [], bytes()

    # If we found the start of a packet then realign to it
    data = data[idx:]

    # If we don't have enough bytes left then wait for more
    if len(data) < 22:
        return [], data

    # Validate the checksum
    checksum = 0
    for index in range(10):
        checksum = (checksum << 1) + struct.unpack_from('<H', data, offset=index*2)[0]
    checksum = ((checksum & 0x7FFF) + (checksum >> 15)) & 0x7FFF
    valid_checksum = struct.unpack_from('<H', data, offset=20)[0]
    if checksum != valid_checksum:
        print("Lidar checksum failed {} != {}!".format(checksum, valid_checksum))

    readings = []

    # We should have a valid packet at the start of data now.
    starting_angle = (data[1] - 0xA0) / 45 * math.pi
    angles = [starting_angle + x*(math.pi/180)for x in range(4)]
    speed = struct.unpack_from('<H', data, offset=2)[0]
    # Each packet contains four distance readings
    for index in range(4):
        reading = struct.unpack_from('<H', data, offset=4*index+4)[0]
        readings.append({
            "angle": starting_angle + index * (math.pi / 180),
            "invalid": bool(reading & 0x8000),
            "weak": bool(reading & 0x4000),
            "distance": reading & 0x3FFF,
            "strength": struct.unpack_from('<H', data, offset=4*index+6)[0],
        })

    # Return the remaining buffer
    return readings, data[22:]
