#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Contains the source code for defining the obstacle class and function for collision detection along a path
"""
import numpy as np
import matplotlib.pyplot as plt

class Obstacle(object):
    '''
    Obstacle object using AABB min-max coordinate representation
    In addition to the min-max point coordinates, it also stores such values for the collision object given
    a robot of given redius
    '''
    def __init__(self, point_min=[0, 0, 0], point_max=[0, 0, 0], radius=0.05):
        self.point_min_ = point_min
        self.point_max_ = point_max
        self.r_ = radius
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

    # Function for checking collision with a given point
    # return True if there is collision
    def collision_check(self, point):
        if self.collision_x_min_ <= point[0] <= self.collision_x_max_ and \
            self.collision_y_min_ <= point[1] <= self.collision_y_max_ and \
                self.collision_z_min_ <= point[2] <= self.collision_z_max_:
            return True
        else:
            return False


# Function for checking collision along the path connecting two points in a straight line
def collision_check_path(startPose, goalPose, obstacle_array, boundary=None):
    '''
    collision check function for a path (discretization)
    return True if there is collision
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


def plot_three_dee_box(points, ax=None, rgb=(1, 0, 0), opacity=0.6, show=False):
    """
    This function takes 2 3D points, defining a 3D orthogonal box, and plots it.

    :param points: the box-defining points, a set of two 3D coordinates,
    format can be a np.array of shape (2, 3), or an Obstacle object.
    :param ax: the parent plotting environment, if none is provided, one will be created automatically
    :param rgb: the color to use, by default red, format is a tuple of RGB values
    :param opacity: determines the opacity og the box, format is a float, must be between 0 and 1
    :param show: if set to True, the function will automatically plot the box
    :return: None
    """
    if type(points) == Obstacle:
        points = np.array([points.point_min_, points.point_max_])

    if ax is None:
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')

    x = points[:, 0]    # the box's x-coordinate extremities
    y = points[:, 1]    # the box's y-coordinate extremities
    z = points[:, 2]    # the box's z-coordinate extremities

    yy_x, zz_x = np.meshgrid(y, z)      # the grids of points for the planes perpendicular to the x-axis
    xx_y, zz_y = np.meshgrid(x, z)      # the grids of points for the planes perpendicular to the y-axis
    xx_z, yy_z = np.meshgrid(x, y)      # the grids of points for the planes perpendicular to the z-axis

    # plotting all 6 faces of the cuboid
    ax.plot_surface(points[0, 0].reshape(1, 1), yy_x, zz_x, color=rgb, alpha=opacity)
    ax.plot_surface(points[1, 0].reshape(1, 1), yy_x, zz_x, color=rgb, alpha=opacity)
    ax.plot_surface(xx_y, points[0, 1].reshape(1, 1), zz_y, color=rgb, alpha=opacity)
    ax.plot_surface(xx_y, points[1, 1].reshape(1, 1), zz_y, color=rgb, alpha=opacity)
    ax.plot_surface(xx_z, yy_z, points[0, 2].reshape(1, 1), color=rgb, alpha=opacity)
    ax.plot_surface(xx_z, yy_z, points[1, 2].reshape(1, 1), color=rgb, alpha=opacity)

    if show:
        plt.show()
