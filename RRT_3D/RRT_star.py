#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Nov 14 23:44:01 2021

@author: zyz
"""

import os
import sys
# deal with the folders
url1=os.path.abspath(os.path.join(os.getcwd(),"../map/"))
sys.path.append(url1)

import numpy as np
import random
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from collisionCheckingCylinder import collisionCheckingCylinder
from numpy.linalg import norm
from map_cylinder import data_for_cylinder



#%% 
# Definition of Class Node 
class Node:
    def __init__(self, idx, pos):
        self.idx = idx
        self.pos = pos
        self.parent_pos = np.zeros(3)
        self.cost  = 0         # cost from the start to the current node 
        self.parent_idx = 0    # parent's node index


# Definition of the RRT* search function
def RRT_star_path_search(x_start, x_goal, map_boundary): 
    '''
    Given the start position, goal position and map boundary, 
    RRT* method is used to generate a waypoints list, 
    return the path_list.
    '''
    # Set parameters
    dim = 3
    Thr = 5       # threshold for final goal
    Delta = 1     # step for steering

    count = 0 
    pathFind = False

    # Add the first node
    node = Node(0, x_start)  
    node_list = []
    node_list.append(node)

    # Start iteration
    for iter in range(3000):
        # show the progress
        if iter % 200 == 0:
            print('Search progress:', iter)
        # random sampling
        x_rand = np.zeros(dim)
        x_rand[0] = random.random() * map_boundary
        x_rand[1] = random.random() * map_boundary
        x_rand[2] = random.random() * map_boundary
#        ax.plot([x_rand[0]],[x_rand[1]],[x_rand[2]],marker='+',c='k',markersize = 2)

    
        # find the nearest node
        x_nearest = np.zeros(dim)
        nearest_dis = norm(x_goal)
        for i in range(len(node_list)):   
            current_x = node_list[i].pos
            dis = norm(current_x-x_rand)
            if dis < nearest_dis:
                nearest_node = i
                nearest_dis = dis
    
        x_nearest = node_list[nearest_node].pos
        
        # extend to x_new
        x_new = x_nearest + Delta/nearest_dis*(x_rand-x_nearest)
        
        # collision checking
        # if there is a collision, skip the rest and go to the next iteraion
        if not collisionCheckingCylinder(x_nearest,x_new,obstacle_array,map_boundary):
            continue
    
        # Find the neighbor with the lowest cost
        neighbors_list = []
        neighbors_count = 0
        lowest_cost_node = nearest_node
        neighbors_lowest_cost = node_list[lowest_cost_node].cost + norm(x_new-x_nearest)
        for j in range(len(node_list)):
            if j == nearest_node:
                continue
            neighbor_pos = node_list[j].pos
            neighbor_cost = node_list[j].cost + norm(x_new-neighbor_pos)
            dist2xnew = norm(neighbor_pos-x_new)
            # Select the neighbors within the threshold distance 
            dist_thresh = 5
            if dist2xnew < dist_thresh:
                if collisionCheckingCylinder(neighbor_pos,x_new,obstacle_array,map_boundary):
                    neighbors_count += 1
                    neighbors_list.append(j)
                    if neighbor_cost < neighbors_lowest_cost:
                        neighbors_lowest_cost = neighbor_cost
                        lowest_cost_node = j
    
        x_lowest_cost = node_list[lowest_cost_node].pos
        count += 1
    
        # add x_new to the tree, the parent of x_new is x_lowest_cost
        new_node = Node(count, x_new)
        new_node.parent_idx = lowest_cost_node
        new_node.parent_pos = x_lowest_cost
        new_node.cost = node_list[lowest_cost_node].cost + norm(x_new-x_lowest_cost)
        node_list.append(new_node)
    
        # rewire the tree 
        for k in range(neighbors_count):
            neighbor_pos = node_list[k].pos
            neighbor_parent_pos = node_list[k].parent_pos
            ori_cost = node_list[k].cost
            rewire_cost = new_node.cost + norm(x_new-neighbor_pos)
            # if the rewire cost is lower than the original cost, 
            # then rewire the neighbor's parent to x_new
            if rewire_cost < ori_cost:
                node_list[k].parent_pos = x_new
                node_list[k].cost = rewire_cost
                node_list[k].parent_idx = count
#                # delete the old line from neighbor to its previous parent
#                ax.plot([neighbor_pos[0], neighbor_parent_pos[0]], 
#                        [neighbor_pos[1], neighbor_parent_pos[1]],
#                        [neighbor_pos[2], neighbor_parent_pos[2]], 'w')
#                # add the new line from neighbor to its new parent x_new
#                ax.plot([neighbor_pos[0], x_new[0]],
#                        [neighbor_pos[1], x_new[1]],
#                        [neighbor_pos[2], x_new[2]], 'g')
    
    
#        # plot the edge between x_lowest_cost and x_new
#        ax.plot([x_lowest_cost[0], x_new[0]], 
#                [x_lowest_cost[1], x_new[1]],
#                [x_lowest_cost[2], x_new[2]], c='b')

        # check arriving the goal
        if norm(x_new-x_goal) < Thr:
#            ax.plot([x_goal[0], x_new[0]], 
#                    [x_goal[1], x_new[1]],
#                    [x_goal[2], x_new[2]], c='b')
            pathFind = True
            break
    
    
    # back search
    path_list = []
    if pathFind:
        path_list.append(x_goal)
        N = len(node_list)
        path_list.append(node_list[N-1].pos)
        path_index = node_list[N-1].parent_idx
        j = 0
        while 1:
            path_list.append(node_list[path_index].pos)
            path_index = node_list[path_index].parent_idx
            if path_index == 0:
                break
            j += 1
        path_list.append(x_start)
    else:
        print('No path found')

    return path_list

#%% Display the whole process(can be commented all below when importing the search function)

# Initialization and import the obstacle array
x_start = np.array([1,1,1])
x_goal  = np.array([20,20,20])
map_boundary = 20

filename = os.path.join(url1,'obstacles_cylinder.csv')
obstacle_array = np.loadtxt(filename, delimiter=',', dtype=np.float64, skiprows=1)

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

for i in range(obstacle_array.shape[0]):
    center_x, center_y, radius, height_z = obstacle_array[i, :]
    
    Xc,Yc,Zc = data_for_cylinder(center_x,center_y,radius,height_z)
    ax.plot_surface(Xc, Yc, Zc, alpha=0.5)

    
# Draw the start and goal point
ax.plot([x_start[0]],[x_start[1]],[x_start[2]], marker='o',c='r',markersize = 10)
ax.plot([x_goal[0]], [x_goal[1]], [x_goal[2]], marker='o',c='b',markersize = 10)


# Call the RRT search function 
path_list = RRT_star_path_search(x_start, x_goal, map_boundary)


# Draw the final path
path_length = 0
for i in range(len(path_list)-1):
    ax.plot([path_list[i][0], path_list[i+1][0]], 
            [path_list[i][1], path_list[i+1][1]], 
            [path_list[i][2], path_list[i+1][2]], c='r',linewidth=2)
    path_length += norm(path_list[i]-path_list[i+1])

print('Length of path:', round(path_length, 2))

ax.set_title('RRT* Algorithm')
plt.show()

    





    
    