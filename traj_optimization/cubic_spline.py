#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import numpy as np
from numpy.linalg import norm
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
