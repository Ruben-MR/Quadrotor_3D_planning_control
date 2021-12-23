import numpy as np
import matplotlib.pyplot as plt
from numpy.linalg import inv, norm
import scipy.integrate
from scipy.spatial.transform import Rotation
import mpl_toolkits.mplot3d.axes3d as p3
from matplotlib import animation
import forcespro
import casadi
from model.quadrotor import quat_dot, Quadrotor

class MPC_basic(object):
    def __init__(self):
        """
        Parameters of the quadrotor
        """
        self.mass = 0.030               # kg
        self.Ixx = 1.43e-5              # kg*m^2
        self.Iyy = 1.43e-5              # kg*m^2
        self.Izz = 2.89e-5              # kg*m^2
        self.arm_length = 0.046         # meters
        self.rotor_speed_min = 0        # rad/s
        self.rotor_speed_max = 2700     # rad/s, satisfy the constraint of max thrust
        self.k_thrust = 2.3e-08         # N/(rad/s)**2
        self.k_drag = 7.8e-11           # Nm/(rad/s)**2

        # Additional constants.
        self.inertia = np.diag(np.array([self.Ixx, self.Iyy, self.Izz]))  # kg*m^2
        self.inertia_casadi = casadi.SX([[self.Ixx, 0, 0], [0, self.Iyy, 0], [0, 0, self.Izz]])
        self.g = 9.81  # m/s^2

        # Precomputes
        k = self.k_drag / self.k_thrust
        L = self.arm_length
        self.to_TM = np.array([[1, 1, 1, 1],
                               [0, L, 0, -L],
                               [-L, 0, L, 0],
                               [k, -k, k, -k]])
        self.inv_inertia = inv(self.inertia)
        self.inv_inertia_casadi = casadi.SX([[1/self.Ixx, 0, 0],
                                             [0, 1/self.Iyy, 0],
                                             [0, 0, 1/self.Izz]])
        self.weight = np.array([0, 0, -self.mass * self.g])
        self.t_step = 0.01
        self.dt = 0.01

    def continuous_dynamics(self, s, u):
        # rotate_k function (third column of the rotation matrix configured by q)
        rotate_k = casadi.vertcat(2 * (s[6] * s[8] + s[7] * s[9]), 2 * (s[7] * s[8] - s[6] * s[9]), 1 - 2 * (s[6] ** 2 + s[7] ** 2))

        # quat_dot function
        (q0, q1, q2, q3) = (s[6], s[7], s[8], s[9])
        col1 = casadi.vertcat(q3, -q2, q1)
        col2 = casadi.vertcat(q2, q3, -q0)
        col3 = casadi.vertcat(-q1, q0, q3)
        col4 = casadi.vertcat(-q0, -q1, -q2)
        G = casadi.horzcat(col1, col2, col3, col4)
        # G = np.array([[q3, q2, -q1, -q0],
        #               [-q2, q3, q0, -q1],
        #               [q1, -q0, q3, -q2]])
        omega = casadi.vertcat(s[10], s[11], s[12])  # angular velocity
        quaternion = casadi.vertcat(s[6], s[7], s[8], s[9])
        quat_dot = 0.5 * G.T @ omega
        # Augment to maintain unit quaternion.
        quat_err = casadi.sum1(casadi.sum2(quaternion ** 2)) - 1
        quat_err_grad = 2 * quaternion
        quat_dot = quat_dot - quat_err * quat_err_grad

        # Angular velocity derivative
        moment = casadi.vertcat(u[1], u[2], u[3])
        w_dot = self.inv_inertia_casadi @ (moment - casadi.cross(omega, self.inertia_casadi @ omega, 1))

        return casadi.vertcat(s[3], s[4], s[5], (self.weight + u[0] * rotate_k) / self.mass, quat_dot, w_dot)

    def objective(self, z, goal):
        self.goal = np.array([goal[0], goal[1], goal[2], goal[3], goal[4], goal[5], 0, 0, 0, 1, 0, 0, 0])
        return (z[4:] - self.goal).T @ self._Q_goal @ (z[4:]-self.goal) + 0.1 * (z[0]**2 + z[1]**2 + z[2]**2 + z[3]**2)

    def objectiveN(self, z, goal):
        self.goal = np.array([goal[0], goal[1], goal[2], goal[3], goal[4], goal[5], 0, 0, 0, 1, 0, 0, 0])
        return (z[4:] - self.goal).T @ self._Q_goal_N @ (z[4:] - self.goal) + 0.2 * (z[0]**2 + z[1]**2 + z[2]**2 + z[3]**2)

    def control(self, state, goal):
        """
        Sovling NLP prolem in N-step-horizon for optimal control, take the first control input
        """
        # Set initial guess
        # x0 = np.transpose(np.tile(self.inital_guess, (1, self.model.N)))
        # self.problem = {"x0": x0}
        # Set initial condition
        state = Quadrotor._pack_state(state)
        x_current = np.transpose(state)
        self.problem["xinit"] = x_current
        # Set runtime parameters
        self.problem["all_parameters"] = np.transpose(np.tile(goal, (1, self.model.N)))

        # Time to solve the NLP!
        output, exitflag, info = self.solver.solve(self.problem)
        # Make sure the solver has exited properly.
        print("exitflag: ", exitflag)
        # self.inital_guess = output['x01'][:, np.newaxis]
        u = np.zeros(4)
        u[0] = output['x01'][0]
        u[1] = output['x01'][1]
        u[2] = output['x01'][2]
        u[3] = output['x01'][3]

        gama = self.k_drag / self.k_thrust
        cof_temp = np.array([1, 1, 1, 1, 0, self.arm_length, 0, -self.arm_length, -self.arm_length, 0, self.arm_length,
                             0, gama, -gama, gama, -gama]).reshape(4, 4)

        f_i = np.matmul(np.linalg.inv(cof_temp), u)

        cmd_motor_speeds = np.zeros(4)
        cmd_moment = np.zeros(3)

        for i in range(4):
            if f_i[i] < 0:
                f_i[i] = 0
                cmd_motor_speeds[i] = self.rotor_speed_max
            cmd_motor_speeds[i] = np.sqrt(f_i[i] / self.k_thrust)
            if cmd_motor_speeds[i] > self.rotor_speed_max:
                cmd_motor_speeds[i] = self.rotor_speed_max

        cmd_thrust = u[0]
        cmd_moment[0] = u[1]
        cmd_moment[1] = u[2]
        cmd_moment[2] = u[3]

        control_input = {'cmd_rotor_speeds': cmd_motor_speeds,
                         'cmd_thrust': cmd_thrust,
                         'cmd_moment': cmd_moment}

        return control_input
