#!/usr/bin/python3
import struct
import smbus
import time
import json
import sys
import os

COMPASS_ADDR = 0x1e
ACCELEROMETER_ADDR = 0x53
GYROSCOPE_ADDR = 0x68

def record_imu():
  bus = smbus.SMBus(0)

  # Datasheet: http://www.soc-robotics.com/pdfs/HMC5883L.pdf
  compass_config = [
    (0x00, 0b01110100), #Config reg A, 8xavg, 75Hz output rate, normal bias
    (0x01, 0b00100000), #Config reg B, +/- 1.3 Ga
    (0x02, 0b00000000), #Mode reg, continuous mode
  ]
  for i in compass_config:
    bus.write_byte_data(COMPASS_ADDR, *i)

  # Datasheet: http://www.analog.com/media/en/technical-documentation/data-sheets/ADXL345.pdf
  accelerometer_config = [
    (0x2D, 0b00001000), #Wake up, auto sleep disabled, begin measurements
    (0x31, 0b00001000), #Full res, +/- 2g, right-justify
    (0x32, 0b00001111), #Do not use FIFO
    (0x1E, 0), #Zero X offset
    (0x1F, 0), #Zero Y offset
    (0x20, 0), #Zero Z offset
    (0x2C, 0b00001011), #High-power mode, 200Hz samplerate
  ]
  for i in accelerometer_config:
    bus.write_byte_data(ACCELEROMETER_ADDR, *i)

  # Datasheet: https://www.sparkfun.com/datasheets/Sensors/Gyro/PS-ITG-3200-00-01.4.pdf
  gyro_config = [
    (0x15, 0x10), #Set sample rate divider to /16
    (0x16, 0b00011010), #Low-pass at 20Hz
    (0x17, 0b00000000), #I don't care about interrupts...
    (0x3E, 0b00000000), #Use internal clocking, don't sleep
  ]
  for i in gyro_config:
    bus.write_byte_data(GYROSCOPE_ADDR, *i)

  start_time = time.time()
  inc = 0
  while True:
    inc += 1
    if not(inc % 100):
      now = time.time()
      print("FPS: {}".format(100/(now - start_time)))
      start_time = now
    try:
      status = bus.read_byte_data(COMPASS_ADDR, 0x09) #status reg
      if status % 2:
        #Compass reading is ready
        data = bus.read_i2c_block_data(COMPASS_ADDR, 0x07, 2) #0x03 is start of data registers
        data = bytes(data)
        z = struct.unpack_from('>h', data)
#        print({
#          "src": "compass",
#          "time": time.time(),
#          "x": x,
#          "y": y,
#          "z": z,
#        })
    
      status = bus.read_byte_data(ACCELEROMETER_ADDR, 0x30) #Interrupt reg
      status &= 0b10000000 #DATA_READY
      if status:
        data = bus.read_i2c_block_data(ACCELEROMETER_ADDR, 0x32, 4)
        data = bytes(data)
        x, y = struct.unpack_from('<hh', data)
#        print({
#          "src": "accel",
#          "time": time.time(),
#          "x": x,
#          "y": y,
#          "z": z,
#        })

      status = bus.read_byte_data(GYROSCOPE_ADDR, 0x1A) #Interrupt reg
      if status % 2:
        #gyro reading is ready
        data = bus.read_i2c_block_data(GYROSCOPE_ADDR, 0x21, 2)
        data = bytes(data)
        z = struct.unpack_from('>h', data)
#        print({
#          "src": "gyro",
#          "time": time.time(),
#          "x": x,
#          "y": y,
#          "z": z,
#        })
    except Exception as e:
      print("{}".format(e))
      

print("Capturing data...")
record_imu()
