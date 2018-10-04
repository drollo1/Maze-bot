#!/usr/bin/python3
import matplotlib.pyplot as plt
import scipy.optimize
import scipy.spatial
import numpy as np
import random
import math
import json

GRID_SPACING = 279.
THETA_OFFSET = 3.25

def test_wall(wall, points):
    x0,x1, y0,y1 = wall["bounding_box"]
    x = list(points[:,0] * np.cos(points[:,1]))
    y = list(points[:,0] * np.sin(points[:,1]))
    good = 0
    for i in range(len(x)):
        if x0 < x[i] < x1:
            if y0 < y[i] < y1:
                good += 1
    print(good)
    return good > 50

def test():
    walls = [
        {
            "name": "forward",
            "bounding_box": [-120,120, 130,150] # x0,x1, y0,y1
        },
        {
            "name": "backward",
            "bounding_box": [-120,120, -150,-130] # x0,x1, y0,y1
        },
        {
            "name": "right",
            "bounding_box": [130,150, -120,120] # x0,x1, y0,y1    
        },
        {
            "name": "left",
            "bounding_box": [-150,-130, -120,120] # x0,x1, y0,y1
        }
    ]

    with open("./lidar.json", "r") as INPUT:
        data = json.loads(INPUT.read())
    points = []
    for point in data:
        if not point['invalid']:
            points.append((point['distance'], point['angle'] + THETA_OFFSET))
    points = np.array(points[-360:])

    for wall in walls:
        if test_wall(wall, points):
            print("Found: {}".format(wall["name"]))

    ax = plt.subplot(111, projection='polar')
    c = ax.scatter(points[:,1], points[:,0])
    ax.set_theta_zero_location("E")

    plt.show()

if __name__ == "__main__":
    test()