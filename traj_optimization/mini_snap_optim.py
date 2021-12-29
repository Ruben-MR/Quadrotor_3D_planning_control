from matplotlib import pyplot as plt
from scipy.optimize import minimize
from traj_optimization.min_snap_utils import *
import time


# minimum snap optimization function, time_optimal will perform time optimization rather than distance-based allocation
# act_const will enable actator constraints for time optimization for more aggressive penalty on time. Disclaimer, it
# can really become time consuming.
def min_snap_optimizer_3d(path, penalty, time_optimal=True, act_const=False, check_collision=False, obstacles=None, total_time=10):
    # general parameters of the minimum snap algorithm
    n_order = 7
    collision_free = False

    while collision_free is False:
        n_seg = np.size(path, axis=0) - 1
        # if time is optimal, then ts and solutions of x, y, z are dependent
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

        if check_collision:
            idx = find_collisions(obstacles, pos)
            if len(idx) == 0:
                collision_free = True
            else:
                path = extend_path(path, idx, ts, tstep)
        else:
            collision_free = True

    return pos, vel, acc, jerk, snap, ts


# Minimum snap solver function for time optimization
def minimum_snap_np(path, n_seg, n_order, penalty, time_optimal, act_const):
    # variables for time, x, y, z
    n_var = 3 * n_seg * (n_order + 1) + n_seg
    ts = None

    # initial guess
    x0 = np.full(n_var, 0.1)
    x0[:n_seg] = 1
    x0[n_seg:n_seg * (n_order + 1) + n_seg:(n_order + 1)] = path[:-1, 0]
    x0[n_seg * (n_order + 1) + n_seg:2 * n_seg * (n_order + 1) + n_seg:(n_order + 1)] = path[:-1, 1]
    x0[2 * n_seg * (n_order + 1) + n_seg:3 * n_seg * (n_order + 1) + n_seg:(n_order + 1)] = path[:-1, 2]

    # set the constraints, bounds and additional options for the optimizer
    if act_const:
        con = [{'type': 'eq', 'fun': equal_constraint, 'args': [n_seg, n_order, path, time_optimal, ts]},
               {'type': 'ineq', 'fun': inequal_constraint, 'args': [n_seg, n_order]}]
        opts = {'maxiter': 300,  'eps': 2e-8, 'disp': True}
    else:
        penalty = 3000
        con = [{'type': 'eq', 'fun': equal_constraint, 'args': [n_seg, n_order, path, time_optimal, ts]}]
        opts = {'maxiter': 500,  'eps': 2e-8, 'disp': True}
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


# Solver function for minimum snap, created separately from the previous one to avoid confusion with indexation
def minimum_snap_qp(path, ts, n_seg, n_order, time_optimal):
    # number of variables
    n_var = 3 * n_seg * (n_order + 1)
    # initial guess
    x0 = np.zeros(n_var)

    # set the constraints, bounds and additional options for the optimizer
    con = [{'type': 'eq', 'fun': equal_constraint, 'args': [n_seg, n_order, path, time_optimal, ts]}]
    opts = {'maxiter': 300, 'eps': 2e-8, 'disp': True}

    # solve the problem
    penalty = 0
    solution = minimize(obj_function, x0, args=(n_seg, n_order, penalty, time_optimal, ts),
                        method='SLSQP', constraints=con, options=opts)

    # get the solution
    pos_coef_x = solution.x[:n_seg * (n_order + 1)]
    pos_coef_y = solution.x[n_seg * (n_order + 1): 2 * n_seg * (n_order + 1)]
    pos_coef_z = solution.x[2 * n_seg * (n_order + 1): 3 * n_seg * (n_order + 1)]

    return pos_coef_x, pos_coef_y, pos_coef_z


# Auxiliar __main__ function (only executed when running this file alone) for debugging and testing the algorithm,
# can be used for testing the performance by itself on different paths
if __name__ == "__main__":
    # global variables
    path_points = np.array([[0, 0, 0], [2, 4, 2], [4, 2, 3], [3, 3, 1], [5, 5, 3], [8, 2, 4], [10, 7, 8],
                            [8, 6, 5], [11, 9, 7]])
    # path_points = np.array([[0, 0, 0], [1, 3, 0], [2, 4, 2], [4, 2, 3], [3, 3, 1], [5, 5, 4]])

    # compute the optimal path
    time_optimal = True
    start = time.time()
    position, velocity, acceleration, jerk, snap, times = min_snap_optimizer_3d(path_points, penalty=10000,
                                                                                time_optimal=time_optimal, act_const=False)
    idx, speeds = get_max_actuation(acceleration, jerk, snap)
    end = time.time()
    print("Execution took: ", end-start)
    print('Time distribution:\n', np.round(times, 2))
    print('Maximum commanded rotor speeds: ', speeds, 'at index: ', idx)
    # plot the results
    N = len(velocity[:, 0])
    fig, axs = plt.subplots(3)
    if time_optimal:
        fig.suptitle('Optimal Time')
    else:
        fig.suptitle('Proportional Time')

    axs[0].plot(range(N), position[:, 0])
    axs[0].plot(range(N), position[:, 1])
    axs[0].plot(range(N), position[:, 2])
    axs[0].set_title('pos')

    axs[1].plot(range(N), velocity[:, 0])
    axs[1].plot(range(N), velocity[:, 1])
    axs[1].plot(range(N), velocity[:, 2])
    axs[1].set_title('vel')

    axs[2].plot(range(N), acceleration[:, 0])
    axs[2].plot(range(N), acceleration[:, 1])
    axs[2].plot(range(N), acceleration[:, 2])
    axs[2].set_title('acc')
    plt.show()

    fig = plt.figure()
    ax = fig.add_subplot(projection='3d')
    ax.set_xlim(0, 15)
    ax.set_ylim(-5, 10)
    ax.set_zlim(0, 15)
    ax.scatter(path_points[:, 0], path_points[:, 1], path_points[:, 2])
    ax.plot(position[:, 0], position[:, 1], position[:, 2])

    plt.show()
