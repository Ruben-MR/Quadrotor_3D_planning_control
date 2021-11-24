#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Nov 23 20:14:47 2021

@author: richard
"""
import numpy as np

class Obstacle(object):
    '''
    Obstacle object using AABB representation
    '''
    def __init__(self, point_min=[0, 0, 0], point_max=[0, 0, 0], radius=0.005):
        self.point_min_ = point_min
        self.point_max_ = point_max
        self.r_ = 0.005
        self.x_min_ = self.point_min_[0]
        self.y_min_ = self.point_min_[1]
        self.z_min_ = self.point_min_[2]
        self.x_max_ = self.point_max_[0]
        self.y_max_ = self.point_max_[1]
        self.z_max_ = self.point_max_[2]
        self.collision_x_min_ = self.x_min_-self.r_
        self.collision_y_min_ = self.y_min_-self.r_
        self.collision_z_min_ = self.z_min_-self.r_
        self.collision_x_max_ = self.x_max_+self.r_
        self.collision_y_max_ = self.y_max_+self.r_
        self.collision_z_max_ = self.z_max_+self.r_

    # Function for checking collisioin with a given point
    def collision_check(self, point):
        if self.collision_x_min_ <= point[0] <= self.collision_x_max_ and \
            self.collision_y_min_ <= point[1] <= self.collision_y_max_ and \
                self.collision_z_min_ <= point[2] <= self.collision_z_max_:
            return True
        else:
            return False
    
    def physical_vertices(self):
        self.length_x_ = self.x_max_ - self.x_min_
        self.length_y_ = self.y_max_ - self.y_min_
        self.length_z_ = self.z_max_ - self.z_min_
        vertex_1 = np.array([self.x_min_, self.y_min_, self.z_min_])
        vertex_2 = np.array([self.x_min_, self.y_min_+self.length_y_, self.z_min_])
        vertex_3 = np.array([self.x_min_+self.length_x_, self.y_min_+self.length_y_, self.z_min_])
        vertex_4 = np.array([self.x_min_+self.length_x_, self.y_min_, self.z_min_])
        vertex_5 = np.array([self.x_min_, self.y_min_, self.z_max_])
        vertex_6 = np.array([self.x_min_, self.y_min_+self.length_y_, self.z_max_])
        vertex_7 = np.array([self.x_min_+self.length_x_, self.y_min_+self.length_y_, self.z_max_])
        vertex_8 = np.array([self.x_min_+self.length_x_, self.y_min_, self.z_max_])
        vertices = np.vstack((vertex_1, vertex_2, vertex_3, vertex_4, vertex_5, vertex_6, vertex_7, vertex_8))
        return vertices



def collision_check_path(startPose, goalPose, obstacle_array, boundary=None):
    '''
    collision check function for a path (discretization)
    '''
    startPose = np.array(startPose)
    goalPose = np.array(goalPose)
    n = 100
    direction = (goalPose-startPose)/n
    for i in range(n+1):
        currentPose = startPose + i*direction
        for obstacle in obstacle_array: # obstacle_array should be an array containing list of obstacle objects
            collision_single_obs = obstacle.collision_check(currentPose)
            if collision_single_obs:
                return True
    return False

"""
obs = Obstacle([0, 0, 0], [1, 1, 1])
point = np.array([0.5, 0.5, 2])
print(obs.collision_check(point))
print(obs.physical_vertices())
start = [0.5, 0.5, 0.5]
end = [1, 1, 2]
obs_lst = [obs]
print(collision_check_path(start, end, obs_lst))
"""