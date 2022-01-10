"""
File containing the source code for computing the minimum snap optimization of the path generated by the RRT-star
algorithm
"""

from matplotlib import pyplot as plt
from scipy.optimize import minimize
from traj_optimization.min_snap_utils import *
from simulator_helpers import generate_env, init_simulation
from Obstacle import plot_three_dee_box
import time


def min_snap_optimizer_3d(path, penalty, time_optimal=True, act_const=False, check_collision=False, obstacles=None, total_time=10):
    """
    Main function for the minimum snap optimization. Considers a seventh order polinomial between each pair of
    consecutive points in the path and solves the optimization problem for a set of constraints, including null
    derivatives at the endpoints and derivative continuity at the waypoints.

    Additional constraints such as collision avoidance guarantees or actuator constraints can be applied at the cost of
    increased computational time. Also, the time allocated for each segment of the path can be computed proportional to
    the segment length or optimized in the process.

    :param path: set of 3D points to which the optimization must be applied
    :param penalty: time penalty required for time optimization
    :param time_optimal: boolean determining whether segment time optimization must be carried out or not
    :param act_const: boolean determining whether actuator constraints must be considered or not
    :param check_collision: boolean determining whether collision avoidance guarantees must be considered out or not
    :param obstacles: set of obstacles to be considered for collision avoidance, defined both in the workspace and
    configuration space
    :param total_time: total time required to complete the path if time_optimal is set to False
    :return: numpy arrays with the positions, velocities, accelerations, jerks, snaps (the last two required for
     actuator constraints) and segment times for the given path and options. All defined for a time step of 0.01 seconds
     except for the segment time.
    """
    # general parameters of the minimum snap algorithm
    n_order = 7
    collision_free = False

    while collision_free is False:
        n_seg = np.size(path, axis=0) - 1
        # if time_optimal is set to True, then ts and solutions of x, y, z are dependent and
        if time_optimal:
            ts, pos_coef_x, pos_coef_y, pos_coef_z = minimum_snap_np(path, n_seg, n_order, penalty, time_optimal, act_const)
        else:
            ts = compute_proportional_t(path, total_time, n_seg)
            pos_coef_x, pos_coef_y, pos_coef_z = minimum_snap_qp(path, ts, n_seg, n_order, time_optimal)

        # Obtain the coefficients of the higher order polynomial trajectries
        vel_coef_x, vel_coef_y, vel_coef_z = get_derivative_coef(pos_coef_x, pos_coef_y, pos_coef_z, n_order, 1)
        acc_coef_y,  acc_coef_x, acc_coef_z = get_derivative_coef(vel_coef_x, vel_coef_y, vel_coef_z, n_order, 2)
        jerk_coef_y, jerk_coef_x, jerk_coef_z = get_derivative_coef(acc_coef_x, acc_coef_y, acc_coef_z, n_order, 3)
        snap_coef_y, snap_coef_x, snap_coef_z = get_derivative_coef(jerk_coef_x, jerk_coef_y, jerk_coef_z, n_order, 4)

        # For a given timestep, calculate the values of the position and higher derivatives of the trajectory
        tstep = 0.01
        pos = get_point_values(pos_coef_x, pos_coef_y, pos_coef_z, ts, n_seg, n_order, 0, tstep)
        vel = get_point_values(vel_coef_x, vel_coef_y, vel_coef_z, ts, n_seg, n_order, 1, tstep)
        acc = get_point_values(acc_coef_x, acc_coef_y, acc_coef_z, ts, n_seg, n_order, 2, tstep)
        jerk = get_point_values(jerk_coef_x, jerk_coef_y, jerk_coef_z, ts, n_seg, n_order, 3, tstep)
        snap = get_point_values(snap_coef_x, snap_coef_y, snap_coef_z, ts, n_seg, n_order, 4, tstep)

        # If collision checking is required, verify that all positions are collision-free. Otherwise, find the segments
        # with collisions and add a point halfway. Then recompute the algorithm for the modified path
        if check_collision:
            idx = find_collisions(obstacles, pos)
            if len(idx) == 0:
                collision_free = True
            else:
                print("Collision found, modifying path and recomputing")
                path = extend_path(path, idx, ts, tstep)
                print(path)
        else:
            collision_free = True

    return pos, vel, acc, jerk, snap, ts


def minimum_snap_np(path, n_seg, n_order, penalty, time_optimal, act_const):
    """
    Function for computing the minimum snap algorithm when time optimization is required
    :param path: set of 3D points to which the optimization must be applied
    :param n_seg: Number of segments in the path
    :param n_order: order of the polynomial for the optimization, since we are minimizing snap, 7th order
    :param penalty: boolean determining whether segment time optimization must be carried out or not
    :param time_optimal: the time_optimal boolean, required by the objective function and equality constraints of the
    solver
    :param act_const: boolean determining whether actuator constraints must be considered or not
    :return: optimized segment times of the path and the polynomials of the segments for each of the three coordinates
     of the 3D space
    """
    # variables for time, x, y, z
    n_var = 3 * n_seg * (n_order + 1) + n_seg
    ts = None

    # initial guess
    x0 = np.full(n_var, 0.1)
    x0[:n_seg] = 1
    x0[n_seg:n_seg * (n_order + 1) + n_seg:(n_order + 1)] = path[:-1, 0]
    x0[n_seg * (n_order + 1) + n_seg:2 * n_seg * (n_order + 1) + n_seg:(n_order + 1)] = path[:-1, 1]
    x0[2 * n_seg * (n_order + 1) + n_seg:3 * n_seg * (n_order + 1) + n_seg:(n_order + 1)] = path[:-1, 2]

    # set the constraints, bounds and additional options for the optimizer considering whether actuator constrains are
    # considered or not.
    if act_const:
        con = [{'type': 'eq', 'fun': equal_constraint, 'args': [n_seg, n_order, path, time_optimal, ts]},
               {'type': 'ineq', 'fun': inequal_constraint, 'args': [n_seg, n_order]}]
        opts = {'maxiter': 300,  'eps': 2e-8, 'disp': True}
    else:
        if penalty > 2700:
            penalty = 2700
        con = [{'type': 'eq', 'fun': equal_constraint, 'args': [n_seg, n_order, path, time_optimal, ts]}]
        opts = {'maxiter': 300,  'eps': 2e-8, 'disp': True}
    bnds = bound(n_seg, n_var)

    # solve the problem
    solution = minimize(obj_function, x0, args=(n_seg, n_order, penalty, time_optimal, ts),
                        method='SLSQP', bounds=bnds, constraints=con, options=opts)

    # get the solution
    ts = solution.x[:n_seg]
    pos_coef_x = solution.x[n_seg:n_seg * (n_order + 1) + n_seg]
    pos_coef_y = solution.x[n_seg * (n_order + 1) + n_seg: 2 * n_seg * (n_order + 1) + n_seg]
    pos_coef_z = solution.x[2 * n_seg * (n_order + 1) + n_seg: 3 * n_seg * (n_order + 1) + n_seg]

    return ts, pos_coef_x, pos_coef_y, pos_coef_z


def minimum_snap_qp(path, ts, n_seg, n_order, time_optimal):
    """
    Function for computing the minimum snap algorithm when time optimization is NOT required. No option for actuator
    constraints, since the completion time is predefined.
    :param path: set of 3D points to which the optimization must be applied
    :param ts: allocated time for each segment of the path
    :param n_seg: Number of segments in the path
    :param n_order: order of the polynomial for the optimization, since we are minimizing snap, 7th order
    :param time_optimal: the time_optimal boolean, required by the objective function and equality constraints of the
    solver
    :return: polynomials of the segments for each of the three coordinates of the 3D space
    """
    # number of variables
    n_var = 3 * n_seg * (n_order + 1)
    # initial guess
    x0 = np.zeros(n_var)

    # set the constraints, bounds and additional options for the optimizer
    con = [{'type': 'eq', 'fun': equal_constraint, 'args': [n_seg, n_order, path, time_optimal, ts]}]
    opts = {'maxiter': 500, 'eps': 2e-8, 'disp': True}

    # solve the problem
    penalty = 0
    solution = minimize(obj_function, x0, args=(n_seg, n_order, penalty, time_optimal, ts),
                        method='SLSQP', constraints=con, options=opts)

    # get the solution
    pos_coef_x = solution.x[:n_seg * (n_order + 1)]
    pos_coef_y = solution.x[n_seg * (n_order + 1): 2 * n_seg * (n_order + 1)]
    pos_coef_z = solution.x[2 * n_seg * (n_order + 1): 3 * n_seg * (n_order + 1)]

    return pos_coef_x, pos_coef_y, pos_coef_z


if __name__ == "__main__":
    """
    Auxiliar __main__ function (only executed when running this file alone) for debugging and testing the algorithm,
    can be used for testing the performance by itself on different paths    
    """
    # Create the quadrotor class, controller and other initial values
    env, policy, t, time_step, total_SE, total_energy, penalty = init_simulation(mpc=False)
    #################################################################
    # Define the obstacles, plotting figure and axis and other scenario properties
    scenarios = [0, 1, 5]
    T = 20
    penalty = 2500
    for scenario in scenarios:
        obstacles, fig, ax1, map_boundary, starts, ends = generate_env(scenario)
        ax1.view_init(60, 35)
        #########################################################################
        # global path planning using RRT*
        x_start = starts[0]
        x_goal = ends[0]
        # Load the data from previous test and compute minimum snap
        path_points = np.load('../experiment_data_videos/front_end/RRT_2/RRT_points_scenario_' + str(scenario) +
                              '_num_iter_4000_goal_1.npz')
        print(path_points["simplified_path"])
        start = time.time()
        pos, vel, acc, jerk, snap, ts = min_snap_optimizer_3d(path_points["simplified_path"], penalty=penalty, time_optimal=False, total_time=T,
                                                              check_collision=True, obstacles=obstacles)
        end = time.time()
        print("Execution time: " + str(end-start))
        # Compute the length and duration of the trajectory
        length = 0
        for i in range(np.size(ts, 0)):
            length += ts[i]
        print("Trajectory duration: " + str(length))
        length = 0
        for i in range(1, np.size(pos, 0)):
            length += np.linalg.norm(pos[i, :] - pos[i - 1, :])
        print("Trajectory length: " + str(length))
        # Compute maximum actuation
        _, max_actuation = get_max_actuation(acc, jerk, snap)
        print(max_actuation)
        if any(max_actuation > 2500):
            print("Actuation limits surpassed")
        else:
            print("NO actuation limits surpassed")
        # Save the data
        np.savez('../experiment_data_videos/front_end/traj_generation/mini_snap_scenario_'+str(scenario)+'_T_'+str(T)+'.npz',
                 pos=pos, vel=vel, acc=acc, jerk=jerk, snap=snap, ts=ts)

        # Plot everything and save it
        for box in obstacles:
            plot_three_dee_box(box, ax=ax1)
        ax1.plot(pos[:, 0], pos[:, 1], pos[:, 2])
        plt.savefig('../experiment_data_videos/front_end/traj_generation/min_snap_scenario_' + str(scenario) + '_T_'+str(T)+'.jpg')
        plt.show()

