#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Nov 19 17:06:13 2021

@author: richard
"""
import os
import sys
##################################################################
# deal with the folders
url1=os.path.join(os.getcwd(),"model/")
url2=os.path.join(os.getcwd(),"traj_handles_ro47001/")
url3=os.path.join(os.getcwd(),"map/")
sys.path.append(url1)
sys.path.append(url2)
sys.path.append(url3)
#################################################################
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

#%matplotlib inline
'''
plt.rcParams['figure.figsize'] = 16, 16

filename = 'colliders.csv'
data = np.loadtxt(filename, delimiter=',', dtype=np.float64, skiprows=2)
print(data)
#prints:
#[[-305.  -435.    85.5    5.     5.    85.5]
# [-295.  -435.    85.5    5.     5.    85.5]
# [-285.  -435.    85.5    5.     5.    85.5]
# ..., 
# [ 435.   465.     8.     5.     5.     8. ]
# [ 445.   465.     8.     5.     5.     8. ]
# [ 455.   465.     8.     5.     5.     8. ]]
'''
def create_voxmap(data, voxel_size=5):
    """
    Returns a grid representation of a 3D configuration space
    based on given obstacle data.
    
    The `voxel_size` argument sets the resolution of the voxel map. 
    """
    # minimum and maximum north coordinates
    north_min = np.floor(np.amin(data[:, 0] - data[:, 3]))
    north_max = np.ceil(np.amax(data[:, 0] + data[:, 3]))

    # minimum and maximum east coordinates
    east_min = np.floor(np.amin(data[:, 1] - data[:, 4]))
    east_max = np.ceil(np.amax(data[:, 1] + data[:, 4]))

    # maximum altitude
    alt_max = np.ceil(np.amax(data[:, 2] + data[:, 5]))
    
    # given the minimum and maximum coordinates we can
    # calculate the size of the grid.
    north_size = int(np.ceil(north_max - north_min)) // voxel_size
    east_size = int(np.ceil(east_max - east_min)) // voxel_size
    alt_size = int(alt_max) // voxel_size

    # Create an empty grid
    voxmap = np.zeros((north_size, east_size, alt_size), dtype=np.bool)

    for i in range(data.shape[0]):
        # Filling in the voxels that are part of an obstacle with `True`
        # i.e. grid[0:5, 20:26, 2:7] = True
        north, east, alt, d_north, d_east, d_alt = data[i, :]
        obstacle = [
            int(north - d_north - north_min) // voxel_size,
            int(north + d_north - north_min) // voxel_size,
            int(east - d_east - east_min) // voxel_size,
            int(east + d_east - east_min) // voxel_size,
        ]

        height = int(alt + d_alt) // voxel_size
        voxmap[obstacle[0]:obstacle[1], obstacle[2]:obstacle[3], 0:height] = True

    return voxmap
'''
voxmap = create_voxmap(data, 10)
print(voxmap.shape)
#prints:
# (81, 91, 21)

# plot the 3D grid
fig = plt.figure()
ax = fig.gca(projection='3d')
ax.voxels(voxmap, edgecolor='k')
ax.set_xlim(voxmap.shape[0], 0)
ax.set_ylim(0, voxmap.shape[1])
# add a bit to z-axis height for visualization
ax.set_zlim(0, voxmap.shape[2]+20)

plt.xlabel('North')
plt.ylabel('East')

plt.show()
'''