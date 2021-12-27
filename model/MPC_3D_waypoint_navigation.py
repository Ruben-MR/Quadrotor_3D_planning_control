import numpy as np
import matplotlib.pyplot as plt
from numpy.linalg import inv
from matplotlib import animation
import forcespro
from model.quadrotor import Quadrotor
from model.MPC_basic import MPC_basic


class MPC_waypoint(MPC_basic):
    def __init__(self, N, obstacle = False):
        """
        Parameters from the MPC_basic class
        """
        MPC_basic.__init__(self)
        """
        Parameters for the MPC controller
        """
        self.model = forcespro.nlp.SymbolicModel(N) # create a empty model with time horizon of 50 steps
        # objective (cost function)
        # cost matrix for tracking the goal point
        self._Q_goal = np.diag([
            100, 100, 100,      # x, y, z
            30, 30, 30,         # dx, dy, dz
            10, 10, 10, 10,     # qx, qy, qz, qw
            10, 10, 10])        # r, p, q
        self._Q_goal_N = np.diag([
            200, 200, 200,      # x, y, z
            10, 10, 10,         # dx, dy, dz
            10, 10, 10, 10,     # qx, qy, qz, qw
            10, 10, 10])        # r, p, q

        # cost: distance to the goal
        self.model.objective = self.objective
        # specially deal with the cost for the last stage (terminal cost)
        #self.model.objectiveN = self.objectiveN

        # equality constraints (quadrotor model)
        # z[0:4] action z[4:14] state
        self.model.eq = lambda z: forcespro.nlp.integrate(self.continuous_dynamics, z[4:], z[0:4],
                                                          integrator=forcespro.nlp.integrators.RK4, stepsize=self.dt)
        self.model.E = np.concatenate([np.zeros((13, 4)), np.eye(13)], axis=1)  # inter-stage equality Ek@ zk+1=f(zk,pk)
        #  upper/lower variable bounds lb <= z <= ub
        #  inputs | states
        # thrust moment_x moment_y moment_z x y z dx dy dz qx qy qz qw r p q
        # self.model.lb = np.array([0, -np.inf, -np.inf, -np.inf, -np.inf, -np.inf, -np.inf, -np.inf, -np.inf, -np.inf,
        #                           -np.inf, -np.inf, -np.inf, -np.inf, -np.inf, -np.inf, -np.inf])
        # self.model.ub = np.array([2.5*self.mass*self.g, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf,
        #                           np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf])
        # Lower bounds are parametric (indices not mentioned here are -inf)
        self.model.lbidx = [0, 4, 5, 6]
        # Upper bounds are parametric (indices not mentioned here are +inf)
        self.model.ubidx = [0, 4, 5, 6]

        # General (differentiable) nonlinear inequalities hl <= h(x,p) <= hu
        if obstacle:
            self.model.ineq = lambda z, p: np.array([(z[4] - p[6])**2 + (z[5] - p[7])**2 + (z[6] - p[8])**2])
            self.model.hu = np.array([2500])
            self.model.hl = np.array([0.49])

        # set dimensions of the problem
        self.model.nvar = 17    # number of variables
        self.model.neq = 13     # number of equality constraints
        if obstacle:
            self.model.nh = 1      # number of inequality constraints functions
            self.model.npar = 9  # number of runtime parameters (pos and vel and pos_obstacle)
        else:
            self.model.npar = 6  # number of runtime parameters (pos and vel and pos_obstacle)
        self.model.xinitidx = range(4, 17)  # indices of the state variables
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
        self.codeoptions.sqp_nlp.reg_hessian = 5e-7  # increase this if exitflag=-8
        self.codeoptions.nlp.stack_parambounds = True
        # Creates code for symbolic model formulation given above, then contacts server to generate new solver
        self.solver = self.model.generate_solver(self.codeoptions)
        #self.solver = forcespro.nlp.Solver.from_directory('FORCESNLPsolver/') # use pre-generated solver

        # Set initial guess to start solver
        self.inital_guess = np.zeros([self.model.nvar, 1])
        x0 = np.transpose(np.tile(self.inital_guess, (1, self.model.N)))
        self.problem = {"x0": x0}
        print(self.model)

    def objective(self, z, goal):
        self.goal = np.array([goal[0], goal[1], goal[2], goal[3], goal[4], goal[5], 0, 0, 0, 1, 0, 0, 0])
        return (z[4:] - self.goal).T @ self._Q_goal @ (z[4:]-self.goal) + 0.1 * z[0]**2

    def objectiveN(self, z, goal):
        self.goal = np.array([goal[0], goal[1], goal[2], goal[3], goal[4], goal[5], 0, 0, 0, 1, 0, 0, 0])
        return (z[4:] - self.goal).T @ (10 * self._Q_goal_N) @ (z[4:] - self.goal) + 0.1 * z[0]**2

    def control(self, state, goal, bounding_box_size = 2):
        """
        Sovling NLP prolem in N-step-horizon for optimal control, take the first control input
        """
        # Set initial guess
        x0 = np.transpose(np.tile(self.inital_guess, (1, self.model.N)))
        self.problem = {"x0": x0}
        # Set initial condition
        state = Quadrotor._pack_state(state)
        x_current = np.transpose(state)
        self.problem["xinit"] = x_current
        # Set runtime parameters
        self.problem["all_parameters"] = np.transpose(np.tile(goal, (1, self.model.N)))
        # Set runtime constraints
        self.bounding_box_size = bounding_box_size
        # only bound first 5 steps (since MPC only executes the first control command)
        lb_first = np.tile([0, state[0]-self.bounding_box_size, state[1]-self.bounding_box_size, state[2]-self.bounding_box_size], (15,))
        lb_second = np.tile([0, state[0]-100, state[1]-100, state[2]-100], (self.model.N - 15, ))
        self.problem["lb"] = np.hstack((lb_first, lb_second))
        ub_first = np.tile([2.5*self.mass*self.g, state[0]+self.bounding_box_size, state[1]+self.bounding_box_size, state[2]+self.bounding_box_size], (15, ))
        ub_second = np.tile([2.5*self.mass*self.g, state[0]+100, state[1]+100, state[2]+100], (self.model.N - 15, ))
        self.problem["ub"] = np.hstack((ub_first, ub_second))

        # Time to solve the NLP!
        output, exitflag, info = self.solver.solve(self.problem)
        # Make sure the solver has exited properly.
        print("exitflag: ", exitflag)
        self.inital_guess = output['x01'][:, np.newaxis]
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
#######################################################################################################################
# You can check the results of a single optimization step here, be sure to comment the main function below
#######################################################################################################################
# mpc = MPC()
#
# # Set initial guess to start solver from (here, middle of upper and lower bound)
# x0i = np.zeros(17)
# x0 = np.transpose(np.tile(x0i, (1, mpc.model.N)))
# init_state = np.zeros(13)
# init_state[9] = 1
# problem = {"x0": x0, "xinit": np.transpose(init_state)}
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
#######################################################################################################################


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
    i = 0
    endpoint = np.array([15, 15, 15, 0, 0, 0, 4, 4, 4])
    controller = MPC_waypoint(40, obstacle=True)
    real_trajectory = {'x': [], 'y': [], 'z': []}
    while t < 10:
        print('iteration: ', i)
        action = controller.control(current_state, endpoint, bounding_box_size=1.5)["cmd_rotor_speeds"]
        obs, reward, done, info = env.step(action)
        real_trajectory['x'].append(obs['x'][0])
        real_trajectory['y'].append(obs['x'][1])
        real_trajectory['z'].append(obs['x'][2])
        print("x, y, z:", obs['x'][0], obs['x'][1], obs['x'][2])
        print("action", action)
        print("--------------------------")
        current_state = obs
        t += dt
        i += 1

    fig = plt.figure()
    ax1 = fig.add_subplot(111, projection="3d")  # 3D place for drawing
    ax1.set_xlim3d(0, np.max(endpoint))
    ax1.set_ylim3d(0, np.max(endpoint))
    ax1.set_zlim3d(0, np.max(endpoint))
    real_trajectory['x'] = np.array(real_trajectory['x'])
    real_trajectory['y'] = np.array(real_trajectory['y'])
    real_trajectory['z'] = np.array(real_trajectory['z'])
    point, = ax1.plot([real_trajectory['x'][0]], [real_trajectory['y'][0]], [real_trajectory['z'][0]], 'ro',
                      label='Quadrotor')
    line, = ax1.plot([real_trajectory['x'][0]], [real_trajectory['y'][0]], [real_trajectory['z'][0]],
                     label='Real_Trajectory')
    ax1.scatter([2, 7], [2, 7], [2, 7], color = 'g', s=500)

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
