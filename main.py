#!/usr/bin/python3
import threading
import numpy as np
import time
import math

import movement
import lidar
import imu

GRID_SPACING = 280 # Size of a grid cell in mm, assuming square cells
MAX_HDIST = 75
MAX_WDIST = 20

class Wall():
    def __init__(self, coords=[0,0], horizontal=True, observations=[]):
        # The location of the center of the wall
        self.coords = np.array(coords)
        
        # Horizontal means parallel to the x axis.
        self.horizontal = horizontal

        # self.a and self.b are the points at each end of this wall.
        if horizontal:
            self.a = np.subtract(self.coords, [GRID_SPACING/2, 0])
            self.b = np.add(self.coords, [GRID_SPACING/2, 0])
        else:
            self.a = np.subtract(self.coords, [0, GRID_SPACING/2])
            self.b = np.add(self.coords, [0, GRID_SPACING/2])
            
        # Observations are data points in support or opposition of the existence of this wall
        self.observations = observations
        
        self.present = None
        self.certainty = 0
        
    def observe(self, observation={"type": "present", "certainty": 0}):
        self.observations.append(observation)
        absent = sum([x['certainty'] for x in self.observations if x['type'] == "absent"])
        present = sum([x['certainty'] for x in self.observations if x['type'] == "present"])
        if absent + present < 0.5:
            return False
        recalc = False
        if absent > present:
            if self.present:
                recalc = True
            self.present = False
            self.certainty = absent / (absent + present)
        else:
            if not self.present:
                recalc = True
            self.present = True
            self.certainty = present / (absent + present)
        return recalc

class State():
    def __init__(self):
        # The last time the internal variables of state were updated
        self.update_time = time.time()
        # our current best estimate of our position and rotation
        self.position = np.array([0,0])
        self.grid_cell = (0,0)
        self.rotation = 0

        # current_path is the list of positions we should go to next. Once we are "close enough" to a 
        # position then it may be removed from the list. When we start we can immediately move into the
        # cell in front of us.
        self.current_path = [
            np.array([0,GRID_SPACING,0]),
        ]
        
        # The maze starts with the center of the initial grid cell as (0,0), assuming that we are
        # facing forward into (0,1). This means X is positive to the right, and Y is positive forward.
        
        # There is a shared wall between (0,0) and (0,1)
        shared_wall = Wall(coords=[0, GRID_SPACING/2], horizontal=True, observations=[{"type": "absent", "certainty": 1}])

        self.maze = {
            (0,0): {
                "walls": {
                    "north": shared_wall,
                    "south": Wall(coords=[0, -1*GRID_SPACING/2], horizontal=True, observations=[{"type": "present", "certainty": 1}]),
                    "east": Wall(coords=[GRID_SPACING/2, 0], horizontal=True, observations=[{"type": "present", "certainty": 1}]),
                    "west": Wall(coords=[-1*GRID_SPACING/2, 0], horizontal=True, observations=[{"type": "present", "certainty": 1}]),
                },
            },
            (0,1): {
                "walls": {
                    "north": Wall(coords=[0, 3*GRID_SPACING/2], horizontal=True),
                    "south": shared_wall,
                    "east": Wall(coords=[GRID_SPACING/2, GRID_SPACING], horizontal=True),
                    "west": Wall(coords=[-1*GRID_SPACING/2, GRID_SPACING], horizontal=True),
                }
            }
        }        

    def add_observation(self, reading):
        current_cell = self.maze[self.grid_cell]
        reading['angle'] += self.rotation
        x = reading['distance'] * math.cos(reading['angle']) + self.position[0]
        y = reading['distance'] * math.sin(reading['angle']) + self.position[1]
        recalc = False
        for name, wall in current_cell['walls'].items():
            if wall.horizontal:
                if wall.coords[1] - MAX_HDIST < y < wall.coords[1] + MAX_HDIST:
                    if wall.coords[0] - MAX_WDIST < x < wall.coords[0] + MAX_WDIST:
                        print("Found a wall {}".format(name))
                        if wall.observe({"type": "present", "certainty": 0.01}):
                            recalc = True
            else:
                if wall.coords[1] - MAX_WDIST < y < wall.coords[1] + MAX_WDIST:
                    if wall.coords[0] - MAX_HDIST < x < wall.coords[0] + MAX_HDIST:
                        print("Found a wall {}".format(name))
                        if wall.observe({"type": "present", "certainty": 0.01}):
                            recalc = True
        if recalc:
            print("In need of maze solve")
        
def main():
    state = State()
    
    def process_lidar(readings):
        for reading in readings:
            # decide which wall this reading hit
            # tell the wall(s) whether you observed them
            state.add_observation(reading)


    
    movement_thread = threading.Thread(target=movement.main, args=(state,), name="Movement")
    lidar_thread = threading.Thread(target=lidar.main, args=(state, process_lidar), name="Lidar")
    imu_thread = threading.Thread(target=imu.main, args=(state,), name="IMU")
    movement_thread.daemon = True
    lidar_thread.daemon = True
    imu_thread.daemon = True
    
    lidar_thread.start()
    imu_thread.start()
    time.sleep(2) # Give the lidar a chance to scan things before we move
    movement_thread.start() # Away we go!
    
    while True:
        time.sleep(1)
        print("Here we go!")

        

main()
