import numpy as np

T = 0.95

def get_T_hover():
    return T

def tj_hover(t):

    pos = np.array([0, 0, 0])
    vel = np.array([0, 0, 0])
    acc = np.array([0, 0, 0])
    yaw = 0
    yawdot = 0
    
    state_des = {'pos': pos, 'vel': vel, 'acc': acc, 'yaw': yaw, 'yaw_dot': yawdot}
    
    return state_des
