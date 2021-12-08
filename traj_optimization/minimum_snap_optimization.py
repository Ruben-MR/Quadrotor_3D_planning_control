import numpy as np
from math import factorial
import cvxpy as cp
from matplotlib import pyplot as plt
from Obstacle import Obstacle, plot_three_dee_box


def get_q(n_seg, n_order, ts):
    q = np.zeros((n_seg*(n_order+1), n_seg*(n_order+1)))
    for k in range(n_seg):
        q_k = np.zeros((n_order + 1, n_order + 1))

        # Get the Q matrix for each segment of the trajectory
        for i in range(4, n_order+1):
            for l in range(4, n_order+1):
                q_k[i, l] = i*(i-1)*(i-2)*(i-3)*l*(l-1)*(l-2)*(l-3)*pow(ts[k], (i+l-7))/(i+l-7)
        # Assign it to the corresponding pos of the general Q matrix
        q[(8*k):(8*k+8), (8*k):(8*k+8)] = q_k
    return q


def get_aeq_cont(n_seg, n_order, ts, k):
    aeq_cont = np.zeros((n_seg - 1, n_seg * (n_order + 1)))
    for j in range(n_seg - 1):
        for i in range(k, n_order + 1):
            aeq_cont[j, (n_order+1) * j + i] = factorial(i)*pow(ts[j], i-k)/factorial(i-k)
        aeq_cont[j, (n_order+1) * (j + 1) + k] = -factorial(k)
    return aeq_cont


def get_ab(n_seg, n_order, waypoints, ts, start_cond, end_cond):
    n_all_poly = n_seg * (n_order + 1)
    # set initial and final point constraints
    aeq_start = np.zeros((4, n_all_poly))
    aeq_start[:4, :4] = np.diag([1, 1, 2, 6])
    aeq_end = np.zeros((4, n_all_poly))
    for k in range(4):
        for i in range(k, n_order+1):
            aeq_end[k, -(n_order + 1 - i)] = (factorial(i)*pow(ts[-1], (i-k)))/factorial(i-k)
    beq_start = start_cond
    beq_end = end_cond

    # pos constraints for waypoints
    aeq_wp = np.zeros((n_seg-1, n_all_poly))
    for j in range(n_seg-1):
        aeq_wp[j, 8*(j+1)] = 1
    beq_wp = waypoints[1:-1]

    # pos continuity
    aeq_contp = get_aeq_cont(n_seg, n_order, ts, k=0)

    # velocity continuity
    aeq_contv = get_aeq_cont(n_seg, n_order, ts, k=1)

    # acceleration continuity
    aeq_conta = get_aeq_cont(n_seg, n_order, ts, k=2)

    # jerk continuity
    aeq_contj = get_aeq_cont(n_seg, n_order, ts, k=3)

    aeq_cont = np.vstack((aeq_contp, aeq_contv, aeq_conta, aeq_contj))
    beq_cont = np.zeros(4*(n_seg - 1),)
    aeq = np.vstack((aeq_start, aeq_end, aeq_wp, aeq_cont))
    beq = np.concatenate((beq_start, beq_end, beq_wp, beq_cont))
    return aeq, beq


def compute_proportional_t(path, T, n_seg):
    ts = np.zeros((n_seg,))
    dist = np.zeros((n_seg,))
    dist_sum, t_sum = 0, 0
    for i in range(n_seg):
        dist[i] = np.linalg.norm(path[i+1, :] - path[i, :])
        dist_sum += dist[i]
    for i in range(n_seg-1):
        ts[i] = T*dist[i]/dist_sum
        t_sum += ts[i]
    ts[-1] = T - t_sum
    return ts


def minimum_snap_qp(waypoints, ts, n_seg, n_order):
    start_cond = np.array([waypoints[0], 0, 0, 0])
    end_cond = np.array([waypoints[-1], 0, 0, 0])
    qs = get_q(n_seg, n_order, ts)
    aeq, beq = get_ab(n_seg, n_order, waypoints, ts, start_cond, end_cond)
    x = cp.Variable(n_seg * (n_order + 1))
    prob = cp.Problem(cp.Minimize(cp.quad_form(x, qs)), [aeq @ x == beq])
    prob.solve()
    return np.array(x.value)


def min_snap_optimizer_3d(path, total_time, time_proportional=True):
    n_order = 7
    n_seg = np.size(path, axis=0) - 1
    if time_proportional:
        ts = compute_proportional_t(path, total_time, n_seg)
        print(ts)
    else:
        ts = np.full((n_seg,), 1)

    pos_coef_x = minimum_snap_qp(path[:, 0], ts, n_seg, n_order)
    pos_coef_y = minimum_snap_qp(path[:, 1], ts, n_seg, n_order)
    pos_coef_z = minimum_snap_qp(path[:, 2], ts, n_seg, n_order)

    vel_coef_x = (pos_coef_x.reshape((-1, n_order+1))[:, 1:] * np.arange(start=1, stop=n_order+1).reshape(1, -1)).reshape((-1,))
    vel_coef_y = (pos_coef_y.reshape((-1, n_order+1))[:, 1:] * np.arange(start=1, stop=n_order+1).reshape(1, -1)).reshape((-1,))
    vel_coef_z = (pos_coef_z.reshape((-1, n_order+1))[:, 1:] * np.arange(start=1, stop=n_order+1).reshape(1, -1)).reshape((-1,))

    acc_coef_x = (vel_coef_x.reshape((-1, n_order))[:, 1:] * np.arange(start=1, stop=n_order).reshape(1, -1)).reshape((-1,))
    acc_coef_y = (vel_coef_y.reshape((-1, n_order))[:, 1:] * np.arange(start=1, stop=n_order).reshape(1, -1)).reshape((-1,))
    acc_coef_z = (vel_coef_z.reshape((-1, n_order))[:, 1:] * np.arange(start=1, stop=n_order).reshape(1, -1)).reshape((-1,))

    pos, vel, acc = [], [], []
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

    pos = np.array(pos).reshape((-1, 3))
    vel = np.array(vel).reshape((-1, 3))
    acc = np.array(acc).reshape((-1, 3))
    return pos, vel, acc
