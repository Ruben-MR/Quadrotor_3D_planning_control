#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Nov 14 09:28:14 2021

@author: zyz
"""
import numpy as np
from numpy.linalg import norm

def collisionChecking(startPose, goalPose, obstacle_array, boundary):
    feasible = True
    n = 100
    safety_dis = 20
    direction = (goalPose-startPose)/n
    for i in range(n+1):
        currentPose = startPose + i*direction
        for j in range(obstacle_array.shape[0]):
            if norm(obstacle_array[j,:]-currentPose, 2) < safety_dis or np.min(currentPose)<0 or np.max(currentPose)>boundary:
                feasible = False
                break
        if feasible == False:
            break
    return feasible