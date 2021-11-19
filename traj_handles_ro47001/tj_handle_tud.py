import os
import sys
##################################################################
# deal with the folders
url1=os.path.join(os.getcwd(),"model/")
url2=os.path.join(os.getcwd(),"traj_handles_ro47001/")
url3=os.path.join(os.getcwd(),"map/")
sys.path.append(url1)
sys.path.append(url2)
sys.path.append(url3)
#################################################################
import numpy as np
from tj_from_line import tj_from_line
from numpy import cos, pi, sin

dt = 0.0001
T = 2.1+1.1+2.15+2.15+2.69+2.05+0.95+3.69+2.05

# return the total time for the main loop
def get_T_tud():
    return T

def tj_tud(t):
    
    T1 = 2.1
    T2 = T1+1.1
    T3 = T2+2.15
    T4 = T3+2.15
    T5 = T4+2.69
    T6 = T5+2.05
    T7 = T6+0.95
    T8 = T7+3.69
    T9 = T8+2.05
    
    if t >= 0 and t <= T1:
        pos, vel, acc = tj_from_line(np.array([0, 0, 0]), np.array([0, 0, 7]), T1, t)
    elif t > T1 and t <= T2:
        pos, vel, acc = tj_from_line(np.array([0, 0, 7]), np.array([0, -2, 7]), T2-T1, t-T1)
    elif t > T2 and t <= T3:
        pos, vel, acc = tj_from_line(np.array([0, -2, 7]), np.array([0, 3, 7]), T3-T2, t-T2)
    elif t > T3 and t <= T4:
        pos, vel, acc = tj_from_line(np.array([0, 3, 7]), np.array([0, 3, 2]), T4-T3, t-T3)
    elif t > T4 and t <= T5:
        angle = tj_from_line(pi, 2*pi, T5-T4, t-T4)[0]
        pos = pos_from_angle(angle, 5, 2, 2)
        vel = get_vel_semicircle(pi, 2*pi, T5-T4, t-T4, 5, 2, 2)
        acc = np.array([0, 0, 0])
    elif t > T5 and t <= T6:
        pos, vel, acc = tj_from_line(np.array([0, 7, 2]), np.array([0, 7, 7]), T6-T5, t-T5)
    elif t > T6 and t <= T7:
        pos, vel, acc = tj_from_line(np.array([0, 7, 7]), np.array([0, 8, 7]), T7-T6, t-T6)
    elif t > T7 and t <= T8:
        angle = tj_from_line(pi/2, -pi/2, T8-T7, t-T7)[0]
        pos = pos_from_angle(angle, 8, 3.5, 3.5)
        vel = get_vel_semicircle(pi/2, -pi/2, T8-T7, t-T7, 8, 3.5, 3.5)
        acc = np.array([0, 0, 0])
    elif t > T8 and t <= T9:
        pos, vel, acc = tj_from_line(np.array([0, 8, 0]), np.array([0, 8, 7]), T9-T8, t-T8) 
    elif t > T9:
        pos = np.array([0, 8, 7])
        vel = np.array([0, 0, 0])
        acc = np.array([0, 0, 0])
    state_des = {'pos': pos, 'vel': vel, 'acc': acc, 'yaw': 0, 'yaw_dot': 0}
    
    return state_des

def pos_from_angle(a, y, z, radius):
    
    pos_y = radius * cos(a) + y
    pos_z = radius * sin(a) + z
    pos = np.array([0, pos_y, pos_z])
    
    return pos

def get_vel_semicircle(start_angle, end_angle, T, t, y, z, radius):
    angle1 = tj_from_line(start_angle, end_angle, T, t)[0]
    pos1 = pos_from_angle(angle1, y, z, radius)
    angle2 = tj_from_line(start_angle, end_angle, T, t+dt)[0]
    vel = (pos_from_angle(angle2, y, z, radius) - pos1)/dt
    
    return vel




