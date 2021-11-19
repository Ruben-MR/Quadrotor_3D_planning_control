# -*- coding: utf-8 -*-
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
from scipy.spatial.transform import Rotation
from numpy import sin, cos, arcsin, arctan2, pi
from numpy.linalg import norm


class GeometricControlller:
    def __init__(self):
        
        self.Kp_pos = np.diag([83, 83, 245])
        self.Kd_pos = np.diag([33, 33, 32])
        self.K_R = np.diag([2700, 2700, 540])
        self.K_w = np.diag([200, 200, 40])

        m = 0.030  # weight (in kg) with 5 vicon markers (each is about 0.25g)
        g = 9.81  # gravitational constant
        I = np.array([[1.43e-5, 0, 0],
                      [0, 1.43e-5, 0],
                      [0, 0, 2.89e-5]])  # inertial tensor in m^2 kg
        L = 0.046  # arm length in m
        self.rotor_speed_min = 0
        self.rotor_speed_max = 2700 # rad/s, satisfy the constraint of max thrust
        self.mass = m
        self.inertia = I
        self.invI = np.linalg.inv(I)
        self.g = g
        self.arm_length = L
        self.k_thrust = 2.3e-08
        self.k_drag = 7.8e-11

    def control(self, flat_output, state):
        '''
        :param desired state: pos, vel, acc, yaw, yaw_dot
        :param current state: x, v, q, w
        :return:
        '''
        cmd_motor_speeds = np.zeros((4,))
        cmd_thrust = 0
        cmd_moment = np.zeros((3,))
        
        acc = flat_output['acc'] + self.Kp_pos @ (flat_output['pos']-state['x']) +\
            self.Kd_pos @ (flat_output['vel']-state['v'])
        Fc = self.mass*acc + np.array([0, 0, self.mass*self.g])
        #R = self.quaternion2romat(state['q'])
        R = Rotation.from_quat(state['q']).as_matrix()
        
        # u1
        b3 = R[:, 2] # (3, 1)
        u1 = b3.T @ Fc.reshape([3, 1])
        
        # u2
        b3_c = Fc/norm(Fc) # desired b3
        
        #psi = self.rotmat2euler(R)[2] # yaw angle
        psi = flat_output['yaw']
        a_psi = np.array([np.cos(psi), np.sin(psi), 0])
        cross_b3c_apsi = np.cross(b3_c, a_psi)
        b2_c = cross_b3c_apsi/norm(cross_b3c_apsi)
        
        R_des = np.vstack((np.cross(b2_c,b3_c), b2_c, b3_c)).T # desired rotation matrix        
        ss_matrix = R_des.T @ R - R.T @ R_des # skew symmetric matrix
        veemap = np.array([-ss_matrix[1, 2], ss_matrix[0, 2], -ss_matrix[0, 1]])
        e_R = 0.5*veemap # orientation error
        
        w_des = np.array([0, 0, flat_output['yaw']])
        e_w = state['w'] - w_des # angular velocity 
        
        u2 = self.inertia @ (-self.K_R @ e_R - self.K_w @ e_w)
        
        Len = self.arm_length
        gama = self.k_drag / self.k_thrust
        cof_temp = np.array(
            [1, 1, 1, 1, 0, Len, 0, -Len, -Len, 0, Len, 0, gama, -gama, gama, -gama]).reshape(4, 4)

        u = np.array([u1, u2[0], u2[1], u2[2]])
        F_i = np.matmul(np.linalg.inv(cof_temp), u)
        
        for i in range(4):
            if F_i[i] < 0:
                F_i[i] = 0
                cmd_motor_speeds[i] = self.rotor_speed_max
            cmd_motor_speeds[i] = np.sqrt(F_i[i] / self.k_thrust)
            if cmd_motor_speeds[i] > self.rotor_speed_max:
                cmd_motor_speeds[i] = self.rotor_speed_max

        cmd_thrust = u1
        cmd_moment[0] = u2[0]
        cmd_moment[1] = u2[1]
        cmd_moment[2] = u2[2]
        
        cmd_q = Rotation.as_quat(Rotation.from_matrix(R_des))
        cmd_q = cmd_q / norm(cmd_q)

        control_input = {'cmd_rotor_speeds': cmd_motor_speeds,
                         'cmd_thrust': cmd_thrust,
                         'cmd_moment': cmd_moment,
                         'cmd_q': cmd_q}
        
        return control_input

    # some manually implemented functions just for better understanding
    def eulzxy2rotmat(self, euler):
        phi = euler[0]
        theta = euler[1]
        psi = euler[2]
        
        R = np.array([[cos(psi)*cos(theta) - sin(phi)*sin(psi)*sin(theta), -cos(phi)*sin(psi),\
                       cos(psi)*sin(theta) + cos(theta)*sin(phi)*sin(psi)],\
                       [cos(theta)*sin(psi) + cos(psi)*sin(phi)*sin(theta), cos(phi)*cos(psi),\
                        sin(psi)*sin(theta) - cos(psi)*cos(theta)*sin(phi)],\
                        [-cos(phi)*sin(theta), sin(phi), cos(phi)*cos(theta)]])

        return R
    def quaternion2romat(self, q):
    # q: w i j k
        [q0, q1, q2, q3] = [q[0], q[1], q[2], q[3]]
        R = 2 * np.array([[q0**2+q1**2-0.5, q1*q2-q0*q3, q0*q2+q1*q3],\
                                    [q0*q3+q1*q2, q0**2+q2**2-0.5, q2*q3-q0*q1],\
                                    [q1*q3-q0*q2, q0*q1+q2*q3, q0**2+q3**2-0.5]])
        return R


    def rotmat2euler(self, R):
        if R[2, 1] < 1:
            if R[2, 1] > -1:
                thetaX = arcsin(R[2, 1])
                thetaZ = arctan2(-R[0, 1], R[1, 1])
                thetaY = arctan2(-R[2, 0], R[2, 2])
            else:
                thetaX = -pi/2
                thetaZ = -arctan2(R[0, 2], R[0, 0])
                thetaY = 0
        else:
            thetaX = pi/2;
            thetaZ = arctan2(R[0,2], R[0,0])
            thetaY = 0
        euler = np.array([thetaX, thetaY, thetaZ])
        
        return euler
