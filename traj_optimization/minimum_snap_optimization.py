import numpy as np
import math
import cvxpy as cp
from matplotlib import pyplot as plt


def get_q(n_seg, n_order, ts):
    q = np.zeros((n_seg*(n_order+1), n_seg*(n_order+1)))
    for k in range(n_seg):
        q_k = np.zeros((n_order + 1, n_order + 1))

        # Get the Q matrix for each segment of the trajectory
        for i in range(4, n_order+1):
            for l in range(4, n_order+1):
                q_k[i, l] = i*(i-1)*(i-2)*(i-3)*l*(l-1)*(l-2)*(l-3)*pow(ts[k], (i+l-7))/(i+l-7)
        # Assign it to the corresponding position of the general Q matrix
        q[(8*k):(8*k+8), (8*k):(8*k+8)] = q_k
    return q


def get_ab(n_seg, n_order, waypoints, ts, start_cond, end_cond):
    n_all_poly = n_seg * (n_order + 1)
    # set initial and final point constraints
    aeq_start = np.zeros((4, n_all_poly))
    aeq_start[:4, :4] = np.diag([1, 1, 2, 6])
    aeq_end = np.zeros((4, n_all_poly))
    for k in range(4):
        for i in range(k, n_order+1):
            aeq_end[k, -(n_order + 1 - i)] = (math.factorial(i)*pow(ts[-1], (i-k)))/math.factorial(i-k)
    beq_start = start_cond
    beq_end = end_cond

    # position constraints for waypoints
    aeq_wp = np.zeros((n_seg-1, n_all_poly))
    for j in range(n_seg-1):
        aeq_wp[j, 8*(j+1)] = 1
    beq_wp = waypoints[1:-1]

    # position continuity
    aeq_contp = np.zeros((n_seg - 1, n_all_poly))
    for j in range(n_seg-1):
        for i in range(n_order + 1):
            aeq_contp[j, 8*j+i] = pow(ts[j], i)
        aeq_contp[j, 8*(j+1)] = -1
    beq_contp = np.zeros((n_seg - 1,))

    # velocity continuity
    aeq_contv = np.zeros((n_seg - 1, n_all_poly))
    for j in range(n_seg - 1):
        for i in range(1, n_order + 1):
            aeq_contv[j, 8 * j + i] = math.factorial(i)*pow(ts[j], i-1)/math.factorial(i-1)
        aeq_contv[j, 8 * (j + 1) + 1] = -1
    beq_contv = np.zeros((n_seg - 1,))

    # acceleration continuity
    aeq_conta = np.zeros((n_seg - 1, n_all_poly))
    for j in range(n_seg - 1):
        for i in range(2, n_order + 1):
            aeq_conta[j, 8 * j + i] = math.factorial(i) * pow(ts[j], i-2) / math.factorial(i - 2)
        aeq_conta[j, 8 * (j + 1) + 2] = -2
    beq_conta = np.zeros((n_seg - 1,))

    # jerk continuity
    aeq_contj = np.zeros((n_seg - 1, n_all_poly))
    for j in range(n_seg - 1):
        for i in range(3, n_order + 1):
            aeq_contj[j, 8 * j + i] = math.factorial(i) * pow(ts[j], i-3) / math.factorial(i - 3)
        aeq_contj[j, 8 * (j + 1) + 3] = -6
    beq_contj = np.zeros((n_seg - 1,))

    aeq_cont = np.vstack((aeq_contp, aeq_contv, aeq_conta, aeq_contj))
    beq_cont = np.concatenate((beq_contp, beq_contv, beq_conta, beq_contj))
    aeq = np.vstack((aeq_start, aeq_end, aeq_wp, aeq_cont))
    beq = np.concatenate((beq_start, beq_end, beq_wp, beq_cont))
    return aeq, beq


def compute_proportional_t(waypoints, T, n_seg):
    ts = np.zeros((n_seg,))
    dist = np.zeros((n_seg,))
    dist_sum, t_sum = 0, 0
    for i in range(n_seg):
        dist[i] = np.linalg.norm(waypoints[i+1, :] - waypoints[i, :])
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
    return x.value


path = np.array([[0, 0], [1, 4], [2, 2], [4, 6]])
n_order = 7
n_seg = np.size(path, axis=0) - 1
#ts = compute_proportional_t(path, 25, n_seg)
ts = np.full((n_seg,), 1)
poly_coef_x = minimum_snap_qp(path[:, 0], ts, n_seg, n_order)
poly_coef_y = minimum_snap_qp(path[:, 1], ts, n_seg, n_order)
print(np.sum(poly_coef_y[-9:-1]))
Xn = []
Yn = []
tstep = 0.01

for i in range(n_seg):
    Pxi = poly_coef_x[(8*i):(8*(i+1))]
    Pyi = poly_coef_y[(8*i):(8*(i+1))]
    for t in np.arange(0, ts[i], tstep):
        Xn.append(np.polyval(Pxi[::-1], t))
        Yn.append(np.polyval(Pyi[::-1], t))

plt.scatter(path[:, 0], path[:, 1])
plt.plot(Xn, Yn)
plt.show()
