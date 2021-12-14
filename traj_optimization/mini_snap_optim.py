from matplotlib import pyplot as plt
from scipy.optimize import minimize
import cvxpy as cp
from min_snap_utils import *


# Nonlinear programming based (optimal time allocation) minimum snap
def minimum_snap_np(path, n_seg, n_order, penalty):
    # variables for time, x, y, z
    n_var = 3 * n_seg * (n_order + 1) + n_seg

    # initial guess
    x0 = np.zeros(n_var)
    x0[:n_seg] = 1

    # set the constraints, bounds and additional options for the optimizer
    con = {'type': 'eq', 'fun': equal_constraint, 'args': [n_seg, n_order, path]}
    bnds = bound(n_seg, n_var)
    opts = {'maxiter': 150, 'eps': 2e-8}

    # solve the problem
    solution = minimize(obj_function, x0, args=(n_seg, n_order, penalty), method='SLSQP', bounds=bnds, constraints=con,
                        options=opts)

    # get the solution
    ts = solution.x[:n_seg]
    pos_coef_x = solution.x[n_seg:n_seg * (n_order + 1) + n_seg]
    pos_coef_y = solution.x[n_seg * (n_order + 1) + n_seg: 2 * n_seg * (n_order + 1) + n_seg]
    pos_coef_z = solution.x[2 * n_seg * (n_order + 1) + n_seg: 3 * n_seg * (n_order + 1) + n_seg]

    return ts, pos_coef_x, pos_coef_y, pos_coef_z


# Quadratic programming based minimum snap (given proportional times)
def minimum_snap_qp(waypoints, ts, n_seg, n_order):
    # number of variables
    n_var = n_seg * (n_order + 1)
    # initial guess
    x0 = np.zeros(n_var)

    # set the constraints, bounds and additional options for the optimizer
    con = {'type': 'eq', 'fun': equal_constraint_normal, 'args': [n_seg, n_order, waypoints, ts]}
    opts = {'maxiter': 150, 'eps': 2e-8}

    # solve the problem
    solution = minimize(obj_function_normal, x0, args=(n_seg, n_order, ts), method='SLSQP', constraints=con,
                        options=opts)
    return np.array(solution.x)

# # Quadratic programming based minimum snap (given segment times)
# def minimum_snap_qp(waypoints, ts, n_seg, n_order):
#     qs = get_q(n_seg, n_order, ts)
#     aeq, beq = get_ab(n_seg, n_order, waypoints, ts)
#     x = cp.Variable(n_seg * (n_order + 1))
#     prob = cp.Problem(cp.Minimize(cp.quad_form(x, qs)), [aeq @ x == beq])
#     prob.solve()
#     return np.array(x.value)


# trajectory optimization
def min_snap_optimizer_3d(path, penalty, time_optimal=True):
    # general parameters of the minimum snap algorithm
    n_order = 7
    n_seg = np.size(path, axis=0) - 1

    # if time is optimal, then ts and solutions of x, y, z are dependent
    if time_optimal:
        ts, pos_coef_x, pos_coef_y, pos_coef_z = minimum_snap_np(path, n_seg, n_order, penalty)
    else:
        # ts = np.full((n_seg,), 1)
        T = 10
        ts = compute_proportional_t(path, T, n_seg)
        pos_coef_x = minimum_snap_qp(path[:, 0], ts, n_seg, n_order)
        pos_coef_y = minimum_snap_qp(path[:, 1], ts, n_seg, n_order)
        pos_coef_z = minimum_snap_qp(path[:, 2], ts, n_seg, n_order)
        
    vel_coef_x = (pos_coef_x.reshape((-1, n_order + 1))[:, 1:] * np.arange(start=1, stop=n_order + 1).reshape(1, -1)).reshape((-1,))
    vel_coef_y = (pos_coef_y.reshape((-1, n_order + 1))[:, 1:] * np.arange(start=1, stop=n_order + 1).reshape(1, -1)).reshape((-1,))
    vel_coef_z = (pos_coef_z.reshape((-1, n_order + 1))[:, 1:] * np.arange(start=1, stop=n_order + 1).reshape(1, -1)).reshape((-1,))

    acc_coef_x = (vel_coef_x.reshape((-1, n_order))[:, 1:] * np.arange(start=1, stop=n_order).reshape(1, -1)).reshape((-1,))
    acc_coef_y = (vel_coef_y.reshape((-1, n_order))[:, 1:] * np.arange(start=1, stop=n_order).reshape(1, -1)).reshape((-1,))
    acc_coef_z = (vel_coef_z.reshape((-1, n_order))[:, 1:] * np.arange(start=1, stop=n_order).reshape(1, -1)).reshape((-1,))

    # The jerk and the snap are required for actuator constraint calculation
    jerk_coef_x = (acc_coef_x.reshape((-1, n_order-1))[:, 1:] * np.arange(start=1, stop=n_order-1).reshape(1, -1)).reshape((-1,))
    jerk_coef_y = (acc_coef_y.reshape((-1, n_order-1))[:, 1:] * np.arange(start=1, stop=n_order-1).reshape(1, -1)).reshape((-1,))
    jerk_coef_z = (acc_coef_z.reshape((-1, n_order-1))[:, 1:] * np.arange(start=1, stop=n_order-1).reshape(1, -1)).reshape((-1,))

    snap_coef_x = (jerk_coef_x.reshape((-1, n_order - 2))[:, 1:] * np.arange(start=1, stop=n_order - 2).reshape(1, -1)).reshape((-1,))
    snap_coef_y = (jerk_coef_y.reshape((-1, n_order - 2))[:, 1:] * np.arange(start=1, stop=n_order - 2).reshape(1, -1)).reshape((-1,))
    snap_coef_z = (jerk_coef_z.reshape((-1, n_order - 2))[:, 1:] * np.arange(start=1, stop=n_order - 2).reshape(1, -1)).reshape((-1,))

    pos, vel, acc, jerk, snap = [], [], [], [], []
    tstep = 0.01

    for i in range(n_seg):
        pxi = pos_coef_x[(8 * i):(8 * (i + 1))].tolist()
        pyi = pos_coef_y[(8 * i):(8 * (i + 1))].tolist()
        pzi = pos_coef_z[(8 * i):(8 * (i + 1))].tolist()

        vxi = vel_coef_x[(7 * i):(7 * (i + 1))].tolist()
        vyi = vel_coef_y[(7 * i):(7 * (i + 1))].tolist()
        vzi = vel_coef_z[(7 * i):(7 * (i + 1))].tolist()

        axi = acc_coef_x[(6 * i):(6 * (i + 1))].tolist()
        ayi = acc_coef_y[(6 * i):(6 * (i + 1))].tolist()
        azi = acc_coef_z[(6 * i):(6 * (i + 1))].tolist()

        jxi = jerk_coef_x[(5 * i):(5 * (i + 1))].tolist()
        jyi = jerk_coef_y[(5 * i):(5 * (i + 1))].tolist()
        jzi = jerk_coef_z[(5 * i):(5 * (i + 1))].tolist()

        sxi = snap_coef_x[(4 * i):(4 * (i + 1))].tolist()
        syi = snap_coef_y[(4 * i):(4 * (i + 1))].tolist()
        szi = snap_coef_z[(4 * i):(4 * (i + 1))].tolist()

        for t in np.arange(0, ts[i], tstep):
            pos.append(np.polyval(pxi[::-1], t))
            pos.append(np.polyval(pyi[::-1], t))
            pos.append(np.polyval(pzi[::-1], t))

            vel.append(np.polyval(vxi[::-1], t))
            vel.append(np.polyval(vyi[::-1], t))
            vel.append(np.polyval(vzi[::-1], t))

            acc.append(np.polyval(axi[::-1], t))
            acc.append(np.polyval(ayi[::-1], t))
            acc.append(np.polyval(azi[::-1], t))

            jerk.append(np.polyval(jxi[::-1], t))
            jerk.append(np.polyval(jyi[::-1], t))
            jerk.append(np.polyval(jzi[::-1], t))

            snap.append(np.polyval(sxi[::-1], t))
            snap.append(np.polyval(syi[::-1], t))
            snap.append(np.polyval(szi[::-1], t))

    pos = np.array(pos).reshape((-1, 3))
    vel = np.array(vel).reshape((-1, 3))
    acc = np.array(acc).reshape((-1, 3))
    jerk = np.array(jerk).reshape((-1, 3))
    snap = np.array(snap).reshape((-1, 3))
    return pos, vel, acc, jerk, snap, ts


if __name__ == "__main__":
    # global variables

    path_points = np.array([[5., 7., 3.],
                            [10.86112711, 6.12592946, 2.55994633],
                            [13.91474856, 5.87165097, 2.46509981],
                            [14.26590969, 4.75821649, 2.06223386],
                            [12.38218297, 3.92085155, 2.11475727],
                            [10.97662532, 3.23206355, 1.86315257],
                            [9.50077495, 1.94620373, 1.79910651],
                            [7.62894844, 1.24818373, 1.74496482],
                            [7.18167236, 1.23754264, 1.59337815],
                            [5.49010816, 0.97186878, 1.41324015],
                            [4.66375168, 0.55217968, 1.62723241],
                            [3.35235155, 0.62747605, 1.70443546],
                            [1.64982418, 1.60245634, 2.04395953],
                            [0.5, 2.5, 1.5]])
    """
    path_points = np.array([[0, 0, 0], [1, 3, 0], [2, 4, 2], [4, 2, 3], [3, 3, 1], [5, 5, 3], [8, 2, 4],
                            [10, 7, 8], [9, 3, 10], [12, 5, 2]])
    """
    # compute the optimal path
    time_optimal = False
    position, velocity, acceleration, jerk, snap, times = min_snap_optimizer_3d(path_points, penalty=10000, time_optimal=time_optimal)
    print('Time distribution:\n', np.round(times, 2))
    print('Commanded rotor speeds at time 1', get_input_from_ref(acceleration[100, :], jerk[100, :], snap[100, :]))
    print(acceleration[100, :], jerk[100, :], snap[100, :])
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
