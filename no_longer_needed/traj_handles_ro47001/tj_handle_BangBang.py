#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Nov 25 21:44:41 2021

@author: richard
"""
import numpy as np
from numpy import cos, sin, pi
from numpy import sqrt
from tj_from_line import tj_from_line

def tj_bangbang(t, start, end, T):

    if t >= 0 and t <= T:
        pos, vel, acc = tj_from_line(np.array(start), np.array(end), T, t)
    elif t > T:
        pos = np.array(end)
        vel = np.array([0, 0, 0])
        acc = np.array([0, 0, 0])
    
    yaw = 0;
    yawdot = 0;

    state_des = {'pos': pos, 'vel': vel, 'acc': acc, 'yaw': yaw, 'yaw_dot': yawdot}
    
    return state_des