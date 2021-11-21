#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat Nov 20 22:19:56 2021

@author: zyz
"""

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

#filename = 'obstacles_cylinder.csv'
#data = np.loadtxt(filename, delimiter=',', dtype=np.float64, skiprows=1)

'''
print(data)

'''
def data_for_cylinder(center_x,center_y,radius,height_z):
    z = np.linspace(0, height_z, 50)
    theta = np.linspace(0, 2*np.pi, 50)
    theta_grid, z_grid=np.meshgrid(theta, z)
    x_grid = radius*np.cos(theta_grid) + center_x
    y_grid = radius*np.sin(theta_grid) + center_y
    return x_grid,y_grid,z_grid


#fig = plt.figure()
#ax = fig.add_subplot(111, projection='3d')
#
#for i in range(data.shape[0]):
#    center_x, center_y, radius, height_z = data[i, :]
#    
#    Xc,Yc,Zc = data_for_cylinder(center_x,center_y,radius,height_z)
#    ax.plot_surface(Xc, Yc, Zc, alpha=0.5)
#
#
#plt.show()