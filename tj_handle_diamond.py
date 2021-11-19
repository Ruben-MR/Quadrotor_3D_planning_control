import numpy as np
from numpy import sqrt
from tj_from_line import tj_from_line

T = 5.75; # total time of first three edges

# return the total time for the main loop
def get_T_diamond():
    return T+1.85

def tj_diamond(t):

    if t >= 0 and t < T/3:
        pos, vel, acc = tj_from_line(np.array([0, 0, 0]), np.array([0, sqrt(2), sqrt(2)]), T/3, t)
    elif t >= T/3 and t < 2*T/3:
        pos, vel, acc = tj_from_line(np.array([0, sqrt(2), sqrt(2)]), np.array([0, 0, 2*sqrt(2)]), T/3, t-T/3)
    elif t >= 2*T/3 and t < T:
        pos, vel, acc = tj_from_line(np.array([0, 0, 2*sqrt(2)]), np.array([0, -sqrt(2), sqrt(2)]), T/3, t-2*T/3)
    elif t >= T and t <= T+2:
        pos, vel, acc = tj_from_line(np.array([0, -sqrt(2), sqrt(2)]), np.array([1, 0, 0]), 2, t-T)
    elif t > T+1.85:
        pos = np.array([1, 0, 0])
        vel = np.array([0, 0, 0])
        acc = np.array([0, 0, 0])
    
    yaw = 0;
    yawdot = 0;

    state_des = {'pos': pos, 'vel': vel, 'acc': acc, 'yaw': yaw, 'yaw_dot': yawdot}
    
    return state_des