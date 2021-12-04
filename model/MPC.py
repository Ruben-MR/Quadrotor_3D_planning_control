import numpy as np
import cvxpy as cp
import matplotlib.pyplot as plt
import numpy as np
from gym import core, spaces
from numpy.linalg import inv, norm
import scipy.integrate
from scipy.spatial.transform import Rotation
import mpl_toolkits.mplot3d.axes3d as p3
from matplotlib import animation
import forcespro
import casadi

class Quadrotor(core.Env):
    """
    Quadrotor forward dynamics model.
    """
    def __init__(self):
        """
        Parameters of the quadrotor
        """
        self.mass            =  0.030  # kg
        self.Ixx             = 1.43e-5  # kg*m^2
        self.arm_length      = 0.046  # meters
        self.rotor_speed_min = 0  # rad/s
        self.rotor_speed_max = 2500  # rad/s
        self.k_thrust        = 2.3e-08  # N/(rad/s)**2
        self.k_drag          = 7.8e-11   # Nm/(rad/s)**2
        self.g = 9.81 # m/s^2
        # Precomputes
        self.t_step = 0.01

    def reset(self):
        '''
        state is a 6 dimensional vector
        state = [ y z theta dy dz dtheta ]
        dot_state = [ dy dz dtheta ddy ddz ddtheta]
        '''
        self.state = np.zeros(6)
        self.state[0] = 0
        self.state[1] = 0
        self.state[2] = 0
        self.state[3] = 0
        self.state[4] = 0
        self.state[5] = 0
        return self.state

    def step(self, action):
        # Form autonomous ODE for constant inputs and integrate one time step.
        '''
        action is a 2 dimensional vector
        action = [Fz Mx]
        '''
        s = self.state
        def s_dot_fn(t, s):
            return self._s_dot_fn(t, s, action)
        # turn state into list
        '''
        The next state can be obtained through integration
        '''
        sol = scipy.integrate.solve_ivp(s_dot_fn, (0, self.t_step), s, first_step=self.t_step)
        self.state = sol['y'][:,-1]
        reward = 0
        done = 0
        info = {}
        return self.state, reward, done, info

    def _s_dot_fn(self, t, s, u):
        """
        Compute derivative of state for quadrotor given fixed control inputs as
        an autonomous ODE.
        """
        # Pack into vector of derivatives.
        s_dot = np.zeros((6,))
        s_dot[0:3] = s[3:6]
        s_dot[3] = -u[0]/self.mass*np.sin(s[2])
        s_dot[4] = -self.g+u[0]/self.mass*np.cos(s[2])
        s_dot[5] = u[1]/self.Ixx
        return s_dot

class MPC():
    def __init__(self):
        """
        Parameters of the quadrotor
        """
        self.mass            =  0.030  # kg
        self.Ixx             = 1.43e-5  # kg*m^2
        self.arm_length      = 0.046  # meters
        self.rotor_speed_min = 0  # rad/s
        self.rotor_speed_max = 2500  # rad/s
        self.k_thrust        = 2.3e-08  # N/(rad/s)**2
        self.k_drag          = 7.8e-11   # Nm/(rad/s)**2
        self.g = 9.81 # m/s^2
        # Precomputes
        self.t_step = 0.01
        self.dt = 0.01
        self.model = forcespro.nlp.SymbolicModel(50) # create a empty model
        # objective (cost function)
        # cost matrix for tracking the goal point
        self._Q_goal = np.diag([
            100, 100, 10, # y, z, theta
            10, 10, 10]) # dy, dz, dtheta
        self._Q_goal_N = np.diag([
            200, 200, 10, # y, z, theta
            10, 10, 10]) # dy, dz, dtheta
        self.goal = np.array([0.5, 0.5, 0, 0, 0, 0])
        self.model.objective = lambda z: (z[2:] - self.goal).T @ self._Q_goal @ (z[2:]-self.goal) + 0.1 * z[0]**2 + 0.1 * z[1]**2 # cost: distance to the goal
        self.model.objectiveN = lambda z: (z[2:] - self.goal).T @ self._Q_goal_N @ (z[2:]-self.goal) + 0.2 * z[0]**2 + 0.2 * z[1]**2 # specially deal with the cost for the last stage
        #self.model.objective = lambda z: 100 * (z[2]**2 + z[3]**2) # cost: hovering
        # equality constraints (quadrotor model)
        # z[0:2] action z[2:8] state
        self.model.eq = lambda z: forcespro.nlp.integrate(self.continuous_dynamics, z[2:8], z[0:2], integrator=forcespro.nlp.integrators.RK4, stepsize=self.dt)
        self.model.E = np.concatenate([np.zeros((6, 2)), np.eye(6)], axis=1) # inter-stage equality Ek @ zk+1 = f(zk, pk)
        #  upper/lower variable bounds lb <= z <= ub
        #                     inputs         |  states
        #                     thrust  moment     y        z      theta     dy      dz       dtheta
        self.model.lb = np.array([0, -np.inf, -np.inf, -np.inf, -np.deg2rad(40), -np.inf, -np.inf, -np.inf])
        self.model.ub = np.array([2.5*self.mass*self.g, np.inf, np.inf, np.inf, np.deg2rad(40), np.inf, np.inf, np.inf])

        # # General (differentiable) nonlinear inequalities hl <= h(x,p) <= hu
        # model.ineq = lambda z, p: np.array([z[2] ** 2 + z[3] ** 2,  # x^2 + y^2
        #                                     (z[2] - p[0]) ** 2 + (z[3] - p[1]) ** 2])  # (x-p_x)^2 + (y-p_y)^2
        #
        # # Upper/lower bounds for inequalities
        # model.hu = np.array([9, +np.inf])
        # model.hl = np.array([1, 0.7 ** 2])

        # set dimensions of the problem
        # self.model.N = 50 # horizon length
        self.model.nvar = 8 # number of variables
        self.model.neq = 6 # number of equality constraints
        #self.model.nh = 2 # number of inequality constraints functions
        self.model.xinitidx = range(2, 8) # indices of the state variables

        # handle the last stage separately
        # self.model.objectiveN = lambda z: casadi.sumsqr(z[0:2]-np.array([0, 0.5])) # cost: distance to the goal
        # self.model.nvarN = 8 - 2
        # self.model.EN = np.eye(6)
        # self.model.ineqN = lambda z: z[2:]
        # self.model.lbN = self.model.lb[2:]
        # self.model.ubN = self.model.ub[2:]

        # Set solver options
        self.codeoptions = forcespro.CodeOptions('FORCENLPsolver')
        self.codeoptions.maxit = 200  # Maximum number of iterations
        self.codeoptions.printlevel = 0
        self.codeoptions.optlevel = 0  # 0 no optimization, 1 optimize for size, 2 optimize for speed, 3 optimize for size & speed
        self.codeoptions.cleanup = False
        self.codeoptions.timing = 1
        self.codeoptions.nlp.hessian_approximation = 'bfgs'  # when using solvemethod = 'SQP_NLP' and LSobjective, try out 'gauss-newton' here (original: bfgs)
        self.codeoptions.nlp.bfgs_init = 2.5 * np.identity(8)  # initialization of the hessian approximation
        self.codeoptions.solvemethod = "SQP_NLP"
        self.codeoptions.sqp_nlp.maxqps = 1  # maximum number of quadratic problems to be solved
        self.codeoptions.sqp_nlp.reg_hessian = 5e-9  # increase this if exitflag=-8
        # Creates code for symbolic model formulation given above, then contacts server to generate new solver
        self.solver = self.model.generate_solver(self.codeoptions)
        #self.solver = forcespro.nlp.Solver.from_directory('FORCESNLPsolver/') # use pre-generated solver

        # Set initial guess to start solver from (here, middle of upper and lower bound)
        x0i = np.zeros([self.model.nvar, 1])
        x0 = np.transpose(np.tile(x0i, (1, self.model.N)))
        self.problem = {"x0": x0, "xinit": np.transpose(np.zeros(6))}

    def continuous_dynamics(self, s, u):
        return np.array([s[3], s[4], s[5], -u[0] / self.mass * np.sin(s[2]), -self.g + u[0] / self.mass * np.cos(s[2]), u[1] / self.Ixx])

    def control(self, state):
        """
        Sovling NLP prolem in N-step-horizon for optimal control, take the first control input
        """
        x_current = np.transpose(state)
        self.problem["xinit"] = x_current
        # # Set runtime parameters
        # params = np.array(
        #     [-1.5, 1.])  # In this example, the user can change these parameters by clicking into an interactive window
        # problem["all_parameters"] = np.transpose(np.tile(params, (1, model.N)))

        # Time to solve the NLP!
        output, exitflag, info = self.solver.solve(self.problem)
        # Make sure the solver has exited properly.
        #assert exitflag == 1, "bad exitflag"
        # print("FORCES took {} iterations and {} seconds to solve the problem.".format(info.it, info.solvetime))
        action = np.zeros(2)
        action[0] = output['x01'][0]
        action[1] = output['x01'][1]

        return action
#########################################################################################################################
# You can check the results of a single optimization step here, be sure to comment the main function below
#########################################################################################################################
# mpc = MPC()
#
# # Set initial guess to start solver from (here, middle of upper and lower bound)
# x0i = np.zeros(8)
# x0 = np.transpose(np.tile(x0i, (1, mpc.model.N)))
# problem = {"x0": x0, "xinit": np.transpose(np.zeros(6))}
# # # Set runtime parameters
# # params = np.array(
# #     [-1.5, 1.])  # In this example, the user can change these parameters by clicking into an interactive window
# # problem["all_parameters"] = np.transpose(np.tile(params, (1, model.N)))
#
# # Time to solve the NLP!
# output, exitflag, info = mpc.solver.solve(problem)
#
# # Make sure the solver has exited properly.
# assert exitflag == 1, "bad exitflag"
# print("FORCES took {} iterations and {} seconds to solve the problem.".format(info.it, info.solvetime))
#
# print(output)
# print(output['x01'].shape)
#########################################################################################################################
def animate(i):
    line.set_xdata(real_trajectory['x'][:i + 1])
    line.set_ydata(real_trajectory['y'][:i + 1])
    line.set_3d_properties(real_trajectory['z'][:i + 1])
    point.set_xdata(real_trajectory['x'][i])
    point.set_ydata(real_trajectory['y'][i])
    point.set_3d_properties(real_trajectory['z'][i])


if __name__ == '__main__':
    env = Quadrotor()
    current_state = env.reset()
    print("current:", current_state)
    dt = 0.01
    t = 0
    iter = 0
    controller = MPC()
    real_trajectory = {'x': [], 'y': [], 'z': []}
    while (t < 3):
        print('iteration: ', iter)
        action = controller.control(current_state)
        obs, reward, done, info = env.step(action)
        real_trajectory['x'].append(0)
        real_trajectory['y'].append(obs[0])
        real_trajectory['z'].append(obs[1])
        print("y, z:",obs[0],obs[1])
        print("action", action)
        print("--------------------------")
        current_state = obs
        t += dt
        iter += 1
    fig = plt.figure()
    ax1 = p3.Axes3D(fig)  # 3D place for drawing
    ax1.set_xlim3d(-0.2, 0.2)
    ax1.set_ylim3d(-0.5, 0.5)
    ax1.set_zlim3d(0, 1.5)
    real_trajectory['x'] = np.array(real_trajectory['x'])
    real_trajectory['y'] = np.array(real_trajectory['y'])
    real_trajectory['z'] = np.array(real_trajectory['z'])
    point, = ax1.plot([real_trajectory['x'][0]], [real_trajectory['y'][0]], [real_trajectory['z'][0]], 'ro',
                      label='Quadrotor')
    line, = ax1.plot([real_trajectory['x'][0]], [real_trajectory['y'][0]], [real_trajectory['z'][0]],
                     label='Real_Trajectory')

    ax1.set_xlabel('x')
    ax1.set_ylabel('y')
    ax1.set_zlabel('z')
    ax1.set_title('3D animate')
    ax1.view_init(30, 35)
    ax1.legend(loc='lower right')

    ani = animation.FuncAnimation(fig=fig,
                                  func=animate,
                                  frames=len(real_trajectory['x']),
                                  interval=10,
                                  repeat=False,
                                  blit=False)
    plt.show()
