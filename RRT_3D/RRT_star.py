#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys
url1 = os.path.abspath(os.path.join(os.getcwd(), "../map/"))
url2 = os.path.abspath(os.path.join(os.getcwd(), "../box_plotter/"))
sys.path.append(url1)
sys.path.append(url2)
import numpy as np
import matplotlib.pyplot as plt
from numpy.linalg import norm
from box_plotter import plot_three_dee_box
from Obstacle import Obstacle, collision_check_path


# Definition of Class Node
class Node:
    def __init__(self, pos, cost, parent_idx):
        self.pos = pos
        self.cost = cost                # cost from the start to the current node
        self.parent_idx = parent_idx    # parent's node index


# Definition of the RRT* search class
class RRT_star:
    '''
    Given the start position, goal position and map boundary, 
    RRT* method is used to generate a waypoints list, 
    return the path_list.
    '''

    # Class constructor given an initial position
    def __init__(self, x_start, num_iter, obstacles, ax, thr=0.5):
        # Set parameters
        self.num_dim = 3        # number of dimensions to search for
        self.thr = thr          # threshold for final goal
        self.pathFind = False   # boolean for found path indication
        self.neigh_dist = 5     # threshold for termination
        self.num_iter = num_iter
        self.goal_idx = 0
        self.obstacle_array = obstacles
        self.ax = ax
        # Add the first node
        self.node_list = [Node(pos=x_start, cost=0, parent_idx=-1)]

    # Method for adding new paths
    def find_path(self, x_goal, map_boundary):
        # Start iteration
        for iteration in range(self.num_iter):
            if iteration % 100 == 0:  # show the progress
                print('Search iterations:', iteration)

            # get a new sampled point and the index of the closest node in the list
            x_new, idx = self.new_and_closest(map_boundary)

            # check whether the new point is colliding or not
            if idx is None:
                continue
            else:
                # path collision checking, if there is a collision, skip the rest and go to the next iteraion
                if collision_check_path(self.node_list[idx].pos, x_new, self.obstacle_array):
                    continue

            # Rewire the new node for optimal cost and get the close points
            neigh_list = self.rewire_node(x_new, idx, True)

            # Rewire the neighbouring points to the new sample, for existent nodes, pass its position on the list rather
            # than their position
            for j in neigh_list:
                self.rewire_node(j, [], False)

            # check arriving to the goal
            if (not self.pathFind) and norm(x_new - x_goal) < self.thr:
                self.pathFind = True
                # Add the final point to the node list
                self.node_list.append(Node(x_goal,
                                           self.node_list[-1].cost+norm(x_goal - x_new),
                                           len(self.node_list) - 1))
                self.goal_idx = len(self.node_list) - 1

        return self.pathFind

    # Function for obtaining the path
    def get_path(self):
        # back search
        path_list = []
        if self.pathFind:
            # The index of the last element will be equal to the number of elements added
            path_idx = self.goal_idx
            # Iterate backwards appending the nodes and updating the index until the initial one
            while path_idx >= 0:
                path_list.append(self.node_list[path_idx].pos)
                path_idx = self.node_list[path_idx].parent_idx
        else:
            print('No path found')
        return path_list

    # Function for generating a new sample and the index of the closest neighbor
    def new_and_closest(self, map_boundary):
        # random sampling
        x_rand = np.random.uniform(0, 1, 3) * map_boundary

        # check for collision of the sample with an obstacle
        for obstacle in self.obstacle_array:
            if obstacle.collision_check(x_rand):
                return x_rand, None

        # find the nearest node, given that the furthest one will be the goal
        nearest_dis = float('inf')
        for i in range(len(self.node_list)):
            dis = norm(self.node_list[i].pos - x_rand)
            if dis < nearest_dis:
                nearest_node = i
                nearest_dis = dis
        # Return the position of the newly generated random point and the position of the nearest node in the list
        return x_rand, nearest_node

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
                    if not collision_check_path(self.node_list[j].pos, pos, self.obstacle_array):
                        neigh_list.append(j)
                        if neighbor_cost < lowest_cost:
                            lowest_cost = neighbor_cost
                            optimal_neigh = j

            # add x_new to the tree, the parent of x_new is at index optimal_node
            self.node_list.append(Node(pos, lowest_cost, optimal_neigh))
            return neigh_list

        # If the node is existent, check whether the connection with the new node [at index -1] minimizes the cost
        else:
            node_idx = pos
            rewire_cost = self.node_list[-1].cost + norm(self.node_list[node_idx].pos - self.node_list[-1].pos)
            if rewire_cost < self.node_list[node_idx].cost:
                self.node_list[node_idx].cost = rewire_cost
                self.node_list[node_idx].parent_idx = len(self.node_list) - 1

    def plotTree(self):
        for node in range(1, len(self.node_list)):

            parent_idx = self.node_list[node].parent_idx
            self.ax.plot([self.node_list[parent_idx].pos[0], self.node_list[node].pos[0]],
                         [self.node_list[parent_idx].pos[1], self.node_list[node].pos[1]],
                         [self.node_list[parent_idx].pos[2], self.node_list[node].pos[2]])

            self.ax.scatter([self.node_list[node].pos[0]], [self.node_list[node].pos[1]], [self.node_list[node].pos[2]])


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

'''
# Initialization and import the obstacle array in a simple setting
obstacles = [Obstacle([0.5, 1, 1], [1, 1.5, 1.5]), Obstacle([1.5, 1, 1], [2, 1.5, 1.5])]
x_start = np.array([0, 0, 0])
x_goal = np.array([3, 3, 3])
map_boundary = [3, 3, 3]
'''

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Call the RRT search function
RRT = RRT_star(x_start, 3500, obstacles, ax,1)
path_exists = RRT.find_path(x_goal, map_boundary)
print(path_exists)
path_list = RRT.get_path()
#RRT.plotTree()

# Draw the start and goal point
ax.plot([x_start[0]], [x_start[1]], [x_start[2]], marker='o', c='r', markersize=10)
ax.plot([x_goal[0]], [x_goal[1]], [x_goal[2]], marker='o', c='b', markersize=10)

# Draw the final path
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