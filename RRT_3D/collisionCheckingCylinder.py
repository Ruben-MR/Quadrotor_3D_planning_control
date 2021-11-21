#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Nov 14 09:28:14 2021

@author: zyz
"""
import numpy as np
from numpy.linalg import norm

def collisionCheckingCylinder(startPose, goalPose, obstacle_array, boundary):
    feasible = True
    n = 100
    safety_dis = 5
    direction = (goalPose-startPose)/n
    for i in range(n+1):
        currentPose = startPose + i*direction
        for j in range(obstacle_array.shape[0]):
            if ((norm(obstacle_array[j,0:2]-currentPose[0:2]) < obstacle_array[j,2]+safety_dis 
                  and currentPose[2] <= obstacle_array[j,3] ) 
                  or np.min(currentPose)<0 
                  or np.max(currentPose)>boundary):
                feasible = False
                break
        if feasible == False:
            break
    return feasible

#%% Check some small examples(can be commented all below)
#obstacle_array = np.array([[200, 200,  20, 200],
#                           [  0,   0,  30, 100],
#                           [ 80,  70,  25, 150]])
#boundary = 800
#
#startPose = np.array([1,1,1])
#goalPose = np.array([10,10,10])
#print(collisionChecking(startPose, goalPose, obstacle_array, boundary)) # False
#
#startPose = np.array([30,30,30])
#goalPose = np.array([40,40,40])
#print(collisionChecking(startPose, goalPose, obstacle_array, boundary)) # True
#
#startPose = np.array([30,30,101])
#goalPose = np.array([40,40,200])
#print(collisionChecking(startPose, goalPose, obstacle_array, boundary)) # True
