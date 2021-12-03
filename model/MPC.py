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
        self.model = forcespro.nlp.SymbolicModel() # create a empty model
        # objective (cost function)
        self.model.objective = lambda x: casadi.sumsqr(x[0:2]-np.array([0, 0.5])) # cost: distance to the goal
        # equality constraints (quadrotor model)
        # z[0:2] action z[2:8] state
        self.model.eq = lambda z: forcespro.nlp.integrate(self.continuous_dynamics, z[2:8], z[0:2], integrator=forcespro.nlp.integrators.RK4, stepsize=self.dt)
        self.model.E = np.concatenate([np.zeros((6, 2)), np.eye(6)], axis=1) # construction of indices of the left hand side of the dynamical constraints: E @ [u z].T
        #  upper/lower variable bounds lb <= z <= ub
        #                     inputs         |  states
        #                     thrust  moment     y        z      theta     dy      dz       dtheta
        self.model.lb = np.array([0, -np.inf, -np.inf, -np.inf, -np.inf, -np.inf, -np.inf, -np.inf])
        self.model.ub = np.array([2.5*self.mass*self.g, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf])
        # # General (differentiable) nonlinear inequalities hl <= h(x,p) <= hu
        # model.ineq = lambda z, p: np.array([z[2] ** 2 + z[3] ** 2,  # x^2 + y^2
        #                                     (z[2] - p[0]) ** 2 + (z[3] - p[1]) ** 2])  # (x-p_x)^2 + (y-p_y)^2
        #
        # # Upper/lower bounds for inequalities
        # model.hu = np.array([9, +np.inf])
        # model.hl = np.array([1, 0.7 ** 2])

        # set dimensions of the problem
        self.model.N = 50 # horizon length
        self.model.nvar = 8 # number of variables
        self.model.neq = 6 # number of equality constraints
        self.nh = 2 # number of inequality constraints functions
        self.xinitidx = range(2, 8) # indices of the state variables
        # Set solver options
        self.codeoptions = forcespro.CodeOptions('FORCESNLPsolver')
        self.codeoptions.maxit = 400  # Maximum number of iterations
        self.codeoptions.printlevel = 0
        self.codeoptions.optlevel = 0  # 0 no optimization, 1 optimize for size, 2 optimize for speed, 3 optimize for size & speed
        self.codeoptions.nlp.bfgs_init = 3.0 * np.identity(8)  # initialization of the hessian approximation
        self.codeoptions.noVariableElimination = 1.
        # Creates code for symbolic model formulation given above, then contacts server to generate new solver
        self.solver = self.model.generate_solver(self.codeoptions)

    def continuous_dynamics(self, s, u):
        return np.array([s[3], s[4], s[5], -u[0] / self.mass * np.sin(s[2]), -self.g + u[0] / self.mass * np.cos(s[2]), u[1] / self.Ixx])


mpc = MPC()

# Set initial guess to start solver from (here, middle of upper and lower bound)
x0i = np.array([0., 0., 0., 0., 0., 0., 0., 0.])
x0 = np.transpose(np.tile(x0i, (1, mpc.model.N)))

# set initial condition
xinit = np.transpose(np.array([0., 0., 0., 0., 0., 0.]))
xfinal = np.transpose(np.array([0., 2., 0., 0., 0., 0.]))
# problem = {"x0": x0,
#            "xinit": xinit,
#            "xfinal": xfinal}
problem = {"x0": x0}
# # Set runtime parameters
# params = np.array(
#     [-1.5, 1.])  # In this example, the user can change these parameters by clicking into an interactive window
# problem["all_parameters"] = np.transpose(np.tile(params, (1, model.N)))

# Time to solve the NLP!
output, exitflag, info = mpc.solver.solve(problem)

# Make sure the solver has exited properly.
assert exitflag == 1, "bad exitflag"
print("FORCES took {} iterations and {} seconds to solve the problem.".format(info.it, info.solvetime))

print(output)
print(output['x01'].shape)


























#     def control(self, state):
#         """
#         Build discrete linearization model
#         x(t+1) = A @ x(t) + B @ u(t) + C
#         """
#         # A = np.array([[1, 0, -0.004905, 0.01, 0, -0.000001635],
#         #               [0, 1, 0, 0, 0.01, 0],
#         #               [0, 0, 1, 0, 0, 0.01],
#         #               [0, 0, -0.0981, 1, 0, -0.0004905],
#         #               [0, 0, 0, 0, 1, 0],
#         #               [0, 0, 0, 0, 0, 1]])
#         # B = np.array([[0, -0.0002858],
#         #               [0.001667, 0],
#         #               [0, 3.497],
#         #               [0, -0.1143],
#         #               [0.3333, 0],
#         #               [0, 699.3]])
#         # C = np.array([0, -0.0004905, 0, 0, -0.0981, 0])
#         # if state[2] > 1.5:
#         #     state[2] = 1.5
#         # elif state[2] < -1.5:
#         #     state[2] =-1.5
#         x_0 = np.array([state[0], state[1], state[2], state[3], state[4], state[5]])
#         T = 20  # the number of predicted steps
#         x = cp.Variable((6, T + 1))
#         u = cp.Variable((2, T))
#         cost = 0
#         constr = []
#         """
#         It could be solved as a Convex optimization problem
#         Given a target position [yt, zt] = [0,1], [y, z] = [x[0], x[1]]
#         minimize ||[y, z]-[yt, zt]||
#         Moreover, minimize ||[y, z]-[yt, zt]|| + ||[Fz, Mx]|| to save energy
#         """
#
#         for t in range(T):
#             cost += cp.sum_squares(x[0:2, t + 1]-np.array([0, 0.5])) # cost: distance to the goal
#             # print(type(x[:, t].value), x[:, t].value) # before solving optimizatio problem, the type is None
#             # print(type(u[:, t].value), u[:, t].value)
#             # this means we cannot use our model as constraints
#             constr += [x[:, t + 1] == A @ x[:, t] + B @ u[:, t] + C, # constraint: equatio of motion
#             #constr +=  [x[:, t + 1] == step(u[:, t].squeeze(), x[:, t].squeeze())[0],
#                        cp.norm(x[2,t], 'inf') <= 1, # constraint: max theta
#                        u[0, t] <= 0.75,  #3 constraint: acceleration
#                        cp.norm(u[1, t], 'inf') <= 0.072] # constraint: max moment
#         # print(type(A @ x[:, t] + B @ u[:, t] + C)) type is AddExpression object
#         # sums problem objectives and concatenates constraints.
#         constr += [x[:, 0] == x_0] # constraint: initial condition
#         problem = cp.Problem(cp.Minimize(cost), constr)
#         problem.solve(solver=cp.ECOS)
#         action = np.random.randn(2)
#         action[0] = u[0, 0].value
#         action[1] = u[1, 0].value
#         # print(type(x[:, 5].value), x[:, 5]) #after solving optimization problem, the type becomes ndarray
#         # print(type(u[:, 5].value), u[:, 5])
#         return action
#

#
# def animate(i):
#     line.set_xdata(real_trajectory['x'][:i + 1])
#     line.set_ydata(real_trajectory['y'][:i + 1])
#     line.set_3d_properties(real_trajectory['z'][:i + 1])
#     point.set_xdata(real_trajectory['x'][i])
#     point.set_ydata(real_trajectory['y'][i])
#     point.set_3d_properties(real_trajectory['z'][i])
#
#
# if __name__ == '__main__':
#     env = Quadrotor()
#     current_state = env.reset()
#     print("current:", current_state)
#     dt = 0.01
#     t = 0
#     controller = MPC()
#     real_trajectory = {'x': [], 'y': [], 'z': []}
#     while (t < 2):
#         action = controller.control(current_state)
#         obs, reward, done, info = env.step(action)
#         real_trajectory['x'].append(0)
#         real_trajectory['y'].append(obs[0])
#         real_trajectory['z'].append(obs[1])
#         print("y, z:",obs[0],obs[1])
#         print("--------------------------")
#         current_state = obs
#         t += dt
#     fig = plt.figure()
#     ax1 = p3.Axes3D(fig)  # 3D place for drawing
#     ax1.set_xlim3d(-0.2, 0.2)
#     ax1.set_ylim3d(-0.5, 0.5)
#     ax1.set_zlim3d(0, 1.5)
#     real_trajectory['x'] = np.array(real_trajectory['x'])
#     real_trajectory['y'] = np.array(real_trajectory['y'])
#     real_trajectory['z'] = np.array(real_trajectory['z'])
#     point, = ax1.plot([real_trajectory['x'][0]], [real_trajectory['y'][0]], [real_trajectory['z'][0]], 'ro',
#                       label='Quadrotor')
#     line, = ax1.plot([real_trajectory['x'][0]], [real_trajectory['y'][0]], [real_trajectory['z'][0]],
#                      label='Real_Trajectory')
#
#     ax1.set_xlabel('x')
#     ax1.set_ylabel('y')
#     ax1.set_zlabel('z')
#     ax1.set_title('3D animate')
#     ax1.view_init(30, 35)
#     ax1.legend(loc='lower right')
#
#     ani = animation.FuncAnimation(fig=fig,
#                                   func=animate,
#                                   frames=len(real_trajectory['x']),
#                                   interval=10,
#                                   repeat=False,
#                                   blit=False)
#     plt.show()
