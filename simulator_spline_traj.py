#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Nov 30 00:02:21 2021

@author: richard
"""

import os
import sys

##################################################################
# deal with the folders

url1 = os.path.join(os.getcwd(), "model/")
url2 = os.path.join(os.getcwd(), "traj_handles_ro47001/")
sys.path.append(url1)
sys.path.append(url2)

#################################################################
import numpy as np
from matplotlib import pyplot as plt
from matplotlib import animation
from model.quadrotor import Quadrotor
from model.nonlinear_controller import GeometricController
#from traj_handles_ro47001.tj_handle_BangBang import tj_bangbang as tj_handle
from box_plotter import plot_three_dee_box
from Obstacle import Obstacle
from RRT_3D.RRT_star import RRT_star
from scipy.spatial.transform import Rotation
from traj_optimization.cubic_spline import cubic_spline
#################################################################
env = Quadrotor()
# circle trajectory has different initial position
policy = GeometricController()
current_state = env.reset(position=[0, 0, 0])
dt = 0.01
t = 0
time_step = 1e-2
total_SE = 0
total_energy = 0
#################################################################
# set the environment
# Display the whole process(can be commented all below when importing the search function)

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
    
fig = plt.figure()
ax1 = fig.add_subplot(111, projection="3d")
ax1.set_xlim(0, 15)
ax1.set_ylim(-5, 10)
ax1.set_zlim(0, 15)
plt.rcParams['figure.figsize'] = 16, 16

#########################################################################
# test spline traj
# path_list = np.array([[ 3.        ,  7.        ,  3.        ],
#                       [ 6.92606402,  6.4886619 ,  2.78161884],
#                       [10.40370897,  6.34005625,  2.3551076 ],
#                       [13.44698332,  5.71876861,  2.09500267],
#                       [14.33150833,  5.07787231,  2.13299712],
#                       [10.48657387,  3.38083826,  1.92649853],
#                       [ 6.68370723,  1.45014583,  1.46772085],
#                       [ 3.9502406 ,  0.52754665,  0.75817076],
#                       [ 0.        ,  0.        ,  0.        ]])
#########################################################################
# global path planning using RRT*
x_start = np.array([0, 0, 0])
x_goal = np.array([3, 7, 3])
map_boundary = [17, 8, 3]

RRT = RRT_star(x_start, 1000, obstacles, ax1, 1)
path_exists = RRT.find_path(x_goal, map_boundary)
print(path_exists)
path_list = RRT.get_path()
T = 15
pos, vel, acc = cubic_spline(path_list, T)
#########################################################################

real_trajectory = np.zeros((1, 3))
real_orientation = np.zeros((1, 4))
# follow the path
for i in range(len(pos)):
        state_des = {'pos': pos[i], 'vel': vel[i], 'acc': acc[i], 'yaw': 0, 'yaw_dot': 0}
        action = policy.control(state_des, current_state)
        cmd_rotor_speeds = action['cmd_rotor_speeds']
        obs, reward, done, info = env.step(cmd_rotor_speeds)
        # print("--------------------------")
        # print("current:", obs['x'])
        # print('des_position: ', state_des['pos'])
        # print('error: ', np.round(obs['x'] - state_des['pos'], 3))
        # print("time: ", t)
        if i == 0:
            real_trajectory = np.reshape(obs['x'], (1, 3))
            real_orientation = np.reshape(obs['q'], (1, 4))
        else:
            real_trajectory = np.vstack((real_trajectory, np.reshape(obs['x'], (1, 3))))
            real_orientation = np.vstack((real_orientation, np.reshape(obs['q'], (1, 4))))
        current_state = obs
        t += dt
        total_SE += (np.sum((obs['x'] - state_des['pos']) ** 2) * time_step)
        total_energy += (np.sum(cmd_rotor_speeds ** 2) * time_step)

############################################################################
print("Sum of tracking error (integration): ", total_SE)
print("Total time: ", t)
print("Sum of energy consumption (integration)", total_energy)
############################################################################

# Plot the obstacles
for box in obstacles:
    plot_three_dee_box(box, ax=ax1)
# Plot the start and goal points
ax1.plot([x_start[0]], [x_start[1]], [x_start[2]], marker='o', c='r', markersize=10)
ax1.plot([x_goal[0]], [x_goal[1]], [x_goal[2]], marker='o', c='b', markersize=10)
# Plot the final path
path_length = 0
for i in range(len(path_list) - 1):
    ax1.plot([path_list[i][0], path_list[i + 1][0]],
            [path_list[i][1], path_list[i + 1][1]],
            [path_list[i][2], path_list[i + 1][2]], c='b', linewidth=2)
    path_length += np.linalg.norm(path_list[i] - path_list[i + 1])
print('Length of path:', round(path_length, 2))

# Plot the trajectory of the quadrotor
rot = Rotation.from_quat(real_orientation[0, :]).as_matrix()
#print(rot)
rotor_dists = np.array([[0.046*np.sqrt(2), -0.046*np.sqrt(2), 0],
                        [0.046*np.sqrt(2), 0.046*np.sqrt(2), 0],
                        [-0.046*np.sqrt(2), 0.046*np.sqrt(2), 0],
                        [-0.046*np.sqrt(2), -0.046*np.sqrt(2), 0]])
rots = rotor_dists @ rot
rots = rots + real_trajectory[0, :]
points = np.vstack((real_trajectory[0, :], rots))
point, = ax1.plot(points[:, 0], points[:, 1], points[:, 2], 'r.', label='Quadrotor')
line, = ax1.plot([real_trajectory[0, 0]], [real_trajectory[0, 1]], [real_trajectory[0, 2]], 'g',
                 label='Real_Trajectory')

# RRT.plotTree()

ax1.set_xlabel('x')
ax1.set_ylabel('y')
ax1.set_zlabel('z')
ax1.set_title('3D animate')
ax1.view_init(30, 35)
ax1.legend(loc='lower right')


def animate(i):
    line.set_xdata(real_trajectory[:i + 1, 0])
    line.set_ydata(real_trajectory[:i + 1, 1])
    line.set_3d_properties(real_trajectory[:i + 1, 2])
    rot = Rotation.from_quat(real_orientation[i, :]).as_matrix()
    rots = rotor_dists @ rot
    rots = rots + real_trajectory[i, :]
    points = np.vstack((real_trajectory[i, :], rots))
    point.set_xdata(points[:, 0])
    point.set_ydata(points[:, 1])
    point.set_3d_properties(points[:, 2])


ani = animation.FuncAnimation(fig=fig, func=animate, frames=np.size(real_trajectory,0), interval=1, repeat=False,
                              blit=False)
plt.show()
