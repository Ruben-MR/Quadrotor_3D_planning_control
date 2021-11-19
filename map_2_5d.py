#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Nov 19 17:06:13 2021

@author: richard
"""
import time
import numpy as np
import matplotlib.pyplot as plt
from shapely.geometry import Polygon, Point
#%matplotlib inline

plt.rcParams['figure.figsize'] = 12, 12

filename = 'colliders.csv'
data = np.loadtxt(filename, delimiter=',', dtype='Float64', skiprows=2)
np.loadtxt()
print(data)
#prints:
#[[-310.2389   -439.2315     85.5         5.          5.         85.5     ]
# [-300.2389   -439.2315     85.5         5.          5.         85.5     ]
# [-290.2389   -439.2315     85.5         5.          5.         85.5     ]
# ..., 
# [ 257.8061    425.1645      1.75852     1.292725    1.292725    1.944791]
# [ 293.9967    368.3391      3.557666    1.129456    1.129456    3.667319]
# [ 281.5162    354.4156      4.999351    1.053772    1.053772    4.950246]]

# create polygon
def extract_polygons(data):

    polygons = []
    for i in range(data.shape[0]):
        north, east, alt, d_north, d_east, d_alt = data[i, :]
        
        # Extract the 4 corners of each obstacle
        # NOTE: The order of the points needs to be counterclockwise
        # in order to work with the simple angle test
        # Also, `shapely` draws sequentially from point to point.
        # If the area of the polygon in shapely is 0, you've likely got a weird order.
        obstacle = [north - d_north, north + d_north, east - d_east, east + d_east]
        corners = [(obstacle[0], obstacle[2]), (obstacle[0], obstacle[3]), (obstacle[1], 	     				obstacle[3]), (obstacle[1], obstacle[2])]
        
        # Compute the height of the polygon
        height = alt + d_alt

        p = Polygon(corners)
        polygons.append((p, height))

    return polygons

polygons = extract_polygons(data)
print(len(polygons))
#prints: 2926
plt.show()


