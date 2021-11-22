#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat Nov 13 23:17:55 2021

@author: zyz
"""

import numpy as np
import random
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from collisionChecking import collisionChecking
from numpy.linalg import norm
import time

# Initialization
x_start = np.array([1,1])
x_goal  = np.array([700,700])
Thr = 50       # threshold for final goal
Delta = 30     # step for steering
map_boundary = 800
class Node:
    def __init__(self, idx, pos):
        self.idx = idx
        self.pos = pos
        self.parent_pos = np.zeros(2)
        self.cost = 0         # cost from the start to the current node
        self.parent_idx = 0    # parent's node index
node = Node(0, x_start)  #!!!!
node_list = []
node_list.append(node)

# Generate the map and obstacle array
def generate_obstacle_array(num, dim):
    random.seed(10)
    obstacle_array = np.zeros((num, dim))
    for i in range(num):
        for j in range(dim):
            obstacle_array[i,j] = random.randint(0,map_boundary)
    return obstacle_array
num = 20
dim = 2
obstacle_array = generate_obstacle_array(num,dim)

# draw the obstacles
fig, ax = plt.subplots()
ax.scatter(obstacle_array[:,0], obstacle_array[:,1], c='green', alpha=0.6, marker='v')

    
# Start iteration
    
ax.plot(x_start[0],x_start[1],marker='o',c='r',markersize = 10)
ax.plot(x_goal[0],x_goal[1],marker='o',c='b',markersize = 10)

count = 0 #!!!!
pathFind = False

for iter in range(1000):
    # show the progress
    if iter % 100 == 0:
        print(iter)
    # random sampling
    x_rand = np.zeros(dim)
    x_rand[0] = random.random() * map_boundary
    x_rand[1] = random.random() * map_boundary
    ax.plot(x_rand[0],x_rand[1],marker='+',c='k',markersize = 2)

    
    # find the nearest node
    x_nearest = np.zeros(dim)
    nearest_dis = norm(x_goal)
    for i in range(len(node_list)):   #!!!!
        current_x = node_list[i].pos
        dis = norm(current_x-x_rand)
        if dis < nearest_dis:
            nearest_node = i
            nearest_dis = dis
    
    x_nearest = node_list[nearest_node].pos
    
    # extend to x_new
    x_new = x_nearest + Delta/nearest_dis*(x_rand-x_nearest)
    
    # collision checking
    if not collisionChecking(x_nearest,x_new,obstacle_array,map_boundary):
        continue
    
    count += 1
    # add x_new to the tree, the parent of x_new is x_nearest
    new_node = Node(count, x_new)
    new_node.parent_idx = nearest_node
    new_node.parent_pos = x_nearest
    new_node.cost = node_list[nearest_node].cost + Delta
    node_list.append(new_node)
    
    # plot the edge between x_nearest and x_new
    ax.plot([x_nearest[0], x_new[0]] , [x_nearest[1], x_new[1]], c='b')

    # check arriving the goal
    if norm(x_new-x_goal) < Thr:
        ax.plot([x_goal[0], x_new[0]] , [x_goal[1], x_new[1]], c='b')
        pathFind = True
        break
    
    
#    time.sleep(0.05)
#    plt.show()

#plt.show()
    
# back search
if pathFind:
    path_list = []
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
    for i in range(len(path_list)-1):
        ax.plot([path_list[i][0], path_list[i+1][0]] , [path_list[i][1], path_list[i+1][1]], c='r',linewidth=2)

ax.set_title('RRT Algorithm')
plt.show()

    





    
    