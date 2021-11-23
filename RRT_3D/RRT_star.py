#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Nov 14 23:44:01 2021

@author: zyz
"""

import os
import sys
import numpy as np
import matplotlib.pyplot as plt
from numpy.linalg import norm
from map_cylinder import data_for_cylinder

# deal with the folders
url1 = os.path.abspath(os.path.join(os.getcwd(), "../map/"))
sys.path.append(url1)

from collisionCheckingCylinder import collisionCheckingCylinder


# Definition of Class Node
class Node:
    def __init__(self, idx, pos):
        self.idx = idx
        self.pos = pos
        self.parent_pos = np.zeros(3)
        self.cost = 0  # cost from the start to the current node
        self.parent_idx = -1  # parent's node index


# Definition of the RRT* search class
class RRT_star:
    '''
    Given the start position, goal position and map boundary, 
    RRT* method is used to generate a waypoints list, 
    return the path_list.
    '''

    # Class constructor given an initial position
    def __init__(self, x_start):
        # Set parameters
        self.num_dim = 3        # number of dimensions to search for
        self.thr = 5            # threshold for final goal
        self.delta = 1          # step for steering
        self.count = 0          # counter of nodes added
        self.pathFind = False   # boolean for found path indication
        self.neigh_dist = 5     # threshold for termination
        # Add the first node
        self.node_list = [Node(0, x_start)]

    # Method for adding the
    def find_path(self, x_goal, map_boundary):
        # Start iteration
        for iteration in range(3000):
            if iteration % 200 == 0:  # show the progress
                print('Search progress:', iteration)

            # get a new sampled point and the index of the closest node in the list
            x_new, idx = self.new_and_closest(x_goal, map_boundary)

            # collision checking, if there is a collision, skip the rest and go to the next iteraion
            if not collisionCheckingCylinder(self.node_list[idx].pos, x_new, obstacle_array, map_boundary):
                continue

            # Rewire the new node for optimal cost and get the close points
            neigh_list = self.rewire_node(x_new, idx, True)

            # Rewire the neighbouring points to the new sample
            for k in neigh_list:
                self.rewire_node(k, idx, False)

            # check arriving to the goal
            if norm(x_new - x_goal) < self.thr:
                self.pathFind = True
                break
        # Add the final point to the node list
        if self.pathFind:
            self.count += 1
            self.add_node(x_goal, self.count - 1)

        return self.pathFind

    # Function for obtaining the path
    def get_path(self):
        # back search
        path_list = []
        if self.pathFind:
            # The index of the last element will be equal to the number of elements added
            path_idx = self.count
            # Iterate backwards appending the nodes and updating the index until the initial one
            while path_idx >= 0:
                path_list.append(self.node_list[path_idx].pos)
                path_idx = self.node_list[path_idx].parent_idx
        else:
            print('No path found')
        return path_list

    # Function for generating a new sample and the index of the closest neighbor
    def new_and_closest(self, x_goal, map_boundary):
        # random sampling
        x_rand = np.random.uniform(0, 1, 3) * map_boundary
        # find the nearest node
        nearest_dis = norm(x_goal)
        for i in range(len(self.node_list)):
            dis = norm(self.node_list[i].pos - x_rand)
            if dis < nearest_dis:
                nearest_node = i
                nearest_dis = dis
        x_nearest = self.node_list[nearest_node].pos
        # extend to x_new
        x_new = x_nearest + self.delta / nearest_dis * (x_rand - x_nearest)
        return x_new, nearest_node

    # Function for node rewiring, takes a bool to rewire depending on whether a new sample is given or an existing one
    def rewire_node(self, pos, closest_idx, is_new=True):
        if is_new:
            neigh_list = []
            optimal_neigh = closest_idx
            lowest_cost = self.node_list[closest_idx].cost + norm(pos - self.node_list[closest_idx].pos)

            # Iterate over the list of nodes to find those within distance threshold and in collision-free connection
            for j in range(len(self.node_list)):
                if j == closest_idx:
                    continue
                dist2xnew = norm(pos - self.node_list[j].pos)
                neighbor_cost = self.node_list[j].cost + dist2xnew
                if dist2xnew < self.neigh_dist:
                    if collisionCheckingCylinder(self.node_list[j].pos, pos, obstacle_array, map_boundary):
                        neigh_list.append(j)
                        if neighbor_cost < lowest_cost:
                            lowest_cost = neighbor_cost
                            optimal_neigh = j

            # add x_new to the tree, the parent of x_new is at index optimal_node
            self.count += 1
            self.add_node(pos, optimal_neigh)
            return neigh_list

        # If the node is existent, check whether the connection with the new node minimizes the cost
        else:
            node_idx = pos
            rewire_cost = self.node_list[-1].cost + norm(self.node_list[node_idx].pos - self.node_list[-1].pos)
            if rewire_cost < self.node_list[node_idx].cost:
                self.node_list[node_idx].parent_pos = self.node_list[-1].pos
                self.node_list[node_idx].cost = rewire_cost
                self.node_list[node_idx].parent_idx = self.count

    # Function for adding a node to the list
    def add_node(self, pos, parent_idx):
        new_node = Node(self.count, pos)
        new_node.parent_idx = parent_idx
        new_node.parent_pos = self.node_list[parent_idx].pos
        new_node.cost = self.node_list[parent_idx].cost + norm(pos - new_node.parent_pos)
        self.node_list.append(new_node)


# Display the whole process(can be commented all below when importing the search function)

# Initialization and import the obstacle array
x_start = np.array([1, 1, 1])
x_goal = np.array([20, 20, 20])
map_boundary = 20

filename = os.path.join(url1, 'obstacles_cylinder.csv')
obstacle_array = np.loadtxt(filename, delimiter=',', dtype=np.float64, skiprows=1)

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

for i in range(obstacle_array.shape[0]):
    center_x, center_y, radius, height_z = obstacle_array[i, :]

    Xc, Yc, Zc = data_for_cylinder(center_x, center_y, radius, height_z)
    ax.plot_surface(Xc, Yc, Zc, alpha=0.5)

# Draw the start and goal point
ax.plot([x_start[0]], [x_start[1]], [x_start[2]], marker='o', c='r', markersize=10)
ax.plot([x_goal[0]], [x_goal[1]], [x_goal[2]], marker='o', c='b', markersize=10)

# Call the RRT search function
RRT = RRT_star(x_start)
path_exists = RRT.find_path(x_goal, map_boundary)
path_list = RRT.get_path()

# Draw the final path
path_length = 0
for i in range(len(path_list) - 1):
    ax.plot([path_list[i][0], path_list[i + 1][0]],
            [path_list[i][1], path_list[i + 1][1]],
            [path_list[i][2], path_list[i + 1][2]], c='r', linewidth=2)
    path_length += norm(path_list[i] - path_list[i + 1])

print('Length of path:', round(path_length, 2))

ax.set_title('RRT* Algorithm')
plt.show()
