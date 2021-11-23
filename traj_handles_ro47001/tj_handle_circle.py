import os
import sys

##################################################################
# deal with the folders
url1 = os.path.join(os.getcwd(), "model/")
url2 = os.path.join(os.getcwd(), "traj_handles_ro47001/")
url3 = os.path.join(os.getcwd(), "map/")
sys.path.append(url1)
sys.path.append(url2)
sys.path.append(url3)
#################################################################
import numpy as np
from numpy import cos, sin, pi
from tj_from_line import tj_from_line

radius = 5
dt = 0.0001
T = 7.85  # total time you expect to finish the task


# return the total time for the main loop
def get_T_circle():
    return T


def tj_circle(t):
    if t > T:
        pos = np.array([radius, 0, 2.5])
        vel = np.array([0, 0, 0])
        acc = np.array([0, 0, 0])
    else:
        angle = tj_from_line(0, 2 * pi, T, t)[0]
        pos = pos_from_angle(angle)
        vel = get_vel(t)
        acc = (get_vel(t + dt) - get_vel(t)) / dt
    yaw = 0
    yawdot = 0
    state_des = {'pos': pos, 'vel': vel, 'acc': acc, 'yaw': yaw, 'yaw_dot': yawdot}

    return state_des


def pos_from_angle(a):
    pos = np.array([radius * cos(a), radius * sin(a), 2.5 * a / (2 * pi)])
    return pos


def get_vel(t):
    angle1 = tj_from_line(0, 2 * pi, T, t)[0]
    pos1 = pos_from_angle(angle1)
    angle2 = tj_from_line(0, 2 * pi, T, t + dt)[0]
    vel = (pos_from_angle(angle2) - pos1) / dt

    return vel
