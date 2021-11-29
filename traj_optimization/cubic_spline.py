#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys
# deal with the folders
url1 = os.path.abspath(os.path.join(os.getcwd(), "../map/"))
sys.path.append(url1)
url2 = os.path.abspath(os.path.join(os.getcwd(), "../"))
sys.path.append(url2)
url3 = os.path.abspath(os.path.join(os.getcwd(), "../RRT_3D"))
sys.path.append(url3)

import numpy as np
import matplotlib.pyplot as plt
from numpy.linalg import norm
from box_plotter import plot_three_dee_box
from Obstacle import Obstacle
from RRT_star import RRT_star
from scipy.interpolate import CubicSpline

# trajectory optimization--cubic splines
def cubic_spline(path_list, T):
    path_array = np.array(path_list)
    num = path_array.shape[0]
    # allocate the time interval
    dis_list = []
    for i in range(num-1):
        dis = norm(path_array[num-1-i,:]-path_array[num-i-2,:])
        dis_list.append(dis)
    t_array = np.zeros(num)
    for i in range(num-1):
        t_array[i] = sum(dis_list[0:i]) / sum(dis_list) * T
    t_array[num-1] = T
    # get the X, Y, X array in reverse order
    X = path_array[:,0][::-1]
    Y = path_array[:,1][::-1]
    Z = path_array[:,2][::-1]
    
    # The boundary type is set to be 'clamped', which means 
    # the first derivative at curves ends are zero. 
    fx = CubicSpline(t_array, X, bc_type='clamped')
    t_new = np.linspace(0, T, int(T/0.01))
    x_new = fx(t_new)
    vel_x = fx(t_new, 1)
    acc_x = fx(t_new, 2)
        
    fy = CubicSpline(t_array, Y, bc_type='clamped')
    y_new = fy(t_new)
    vel_y = fy(t_new, 1)
    acc_y = fy(t_new, 2)
    
    fz = CubicSpline(t_array, Z, bc_type='clamped')
    z_new = fz(t_new)
    vel_z = fz(t_new, 1)
    acc_z = fz(t_new, 2)
    
    pos = np.vstack((x_new, y_new, z_new)).T   
    vel = np.vstack((vel_x, vel_y, vel_z)).T   
    acc = np.vstack((acc_x, acc_y, acc_z)).T
    
    return pos, vel, acc


# Initialization and import the obstacle array in a real environment
boxes = list()
boxes.append(np.array([[0, 5, 0], [14, 5.3, 3]]))
boxes.append(np.array([[14, 5, 0], [15, 5.3, 2]]))
boxes.append(np.array([[0, 4, 0], [1, 5, 1]]))
boxes.append(np.array([[1.5, 4, 0], [2.5, 5, 1]]))
boxes.append(np.array([[5, 0, 2], [5.3, 5, 3]]))
boxes.append(np.array([[5, 1, 1], [5.3, 4, 2]]))
boxes.append(np.array([[5, 0, 0], [5.3, 4, 1]]))
boxes.append(np.array([[2, 2.5, 0], [5, 2.8, 3]]))

obstacles = list()
for box in boxes:
    obstacles.append(Obstacle(box[0, :], box[1, :]))

x_start = np.array([0, 0, 0])
x_goal = np.array([3, 7, 3])
map_boundary = [17, 8, 3]

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

## get the path from the path planner
#RRT = RRT_star(x_start, 1000, obstacles, ax, 0.7) # change 3500 to 4000
#path_exists = RRT.find_path(x_goal, map_boundary)
#print(path_exists)
#path_list = RRT.get_path()
##RRT.plotTree()

# for iter = 4000, thr = 0.5
path_list = np.array([[ 3.        ,  7.        ,  3.        ],
                      [ 6.92606402,  6.4886619 ,  2.78161884],
                      [10.40370897,  6.34005625,  2.3551076 ],
                      [13.44698332,  5.71876861,  2.09500267],
                      [14.33150833,  5.07787231,  2.13299712],
                      [10.48657387,  3.38083826,  1.92649853],
                      [ 6.68370723,  1.45014583,  1.46772085],
                      [ 3.9502406 ,  0.52754665,  0.75817076],
                      [ 0.        ,  0.        ,  0.        ]])

T = 10 
pos, vel, acc = cubic_spline(path_list, T)
# Draw the start and goal point
ax.plot([x_start[0]], [x_start[1]], [x_start[2]], marker='o', c='r', markersize=10)
ax.plot([x_goal[0]], [x_goal[1]], [x_goal[2]], marker='o', c='b', markersize=10)

ax.plot(pos[:,0], pos[:,1], pos[:,2], c='g', linewidth=2)

path_length = 0
for i in range(len(path_list) - 1):
    ax.plot([path_list[i][0], path_list[i + 1][0]],
            [path_list[i][1], path_list[i + 1][1]],
            [path_list[i][2], path_list[i + 1][2]], c='b', linewidth=2)
    path_length += norm(path_list[i] - path_list[i + 1])

print('Length of path:', round(path_length, 2))

# Plot the obstacles
for box in obstacles:
    plot_three_dee_box(box, ax=ax)

plt.show()
