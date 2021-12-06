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
        # Assign it to the corresponding position of the general Q matrix
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

    # position constraints for waypoints
    aeq_wp = np.zeros((n_seg-1, n_all_poly))
    for j in range(n_seg-1):
        aeq_wp[j, 8*(j+1)] = 1
    beq_wp = waypoints[1:-1]

    # position continuity
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
    return x.value


def min_snap_optimizer_3d(path, T, time_proportional=True):
    n_order = 7
    n_seg = np.size(path, axis=0) - 1
    if time_proportional:
        ts = compute_proportional_t(path, T, n_seg)
    else:
        ts = np.full((n_seg,), 1)

    poly_coef_x = minimum_snap_qp(path[:, 0], ts, n_seg, n_order)
    poly_coef_y = minimum_snap_qp(path[:, 1], ts, n_seg, n_order)
    poly_coef_z = minimum_snap_qp(path[:, 2], ts, n_seg, n_order)

    Xn, Yn, Zn = [], [], []
    tstep = 0.01

    for i in range(n_seg):
        Pxi = poly_coef_x[(8 * i):(8 * (i + 1))]
        Pyi = poly_coef_y[(8 * i):(8 * (i + 1))]
        Pzi = poly_coef_z[(8 * i):(8 * (i + 1))]
        for t in np.arange(0, ts[i], tstep):
            Xn.append(np.polyval(Pxi[::-1], t))
            Yn.append(np.polyval(Pyi[::-1], t))
            Zn.append(np.polyval(Pzi[::-1], t))

    return Xn, Yn, Zn


# path = np.array([[0, 0, 0], [1, 4, 2], [2, 2, 5], [4, 6, 3]])

path = np.array([[ 3.        ,  7.        ,  3.        ],
                [ 6.92606402,  6.4886619 ,  2.78161884],
                [10.40370897,  6.34005625,  2.3551076 ],
                [13.44698332,  5.71876861,  2.09500267],
                [14.33150833,  5.07787231,  2.13299712],
                [10.48657387,  3.38083826,  1.92649853],
                [ 6.68370723,  1.45014583,  1.46772085],
                [ 3.9502406 ,  0.52754665,  0.75817076],
                [ 0.        ,  0.        ,  0.        ]])

Xn, Yn, Zn = min_snap_optimizer_3d(path, 1, False)

# Initialization and import the obstacle array in a real environment
boxes = list()
boxes.append(np.array([[0, 5, 0], [14, 5.3, 3]]))
boxes.append(np.array([[14, 5, 0], [15, 5.3, 2]]))
boxes.append(np.array([[0, 4, 0], [1, 5, 1]]))
boxes.append(np.array([[1.5, 4, 0], [2.5, 5, 1]]))
boxes.append(np.array([[5, 0, 2], [5.3, 5, 3]]))
boxes.append(np.array([[5, 1, 1], [5.3, 4, 2]]))
boxes.append(np.array([[5, 0, 0], [5.3, 4, 1]]))
boxes.append(np.array([[2, 2.5, 0], [5, 2.8, 3]]))

obstacles = list()
for box in boxes:
    obstacles.append(Obstacle(box[0, :], box[1, :]))

fig = plt.figure()
ax = fig.add_subplot(projection='3d')
ax.scatter(path[:, 0], path[:, 1], path[:, 2])
ax.plot(Xn, Yn, Zn)
for box in obstacles:
    plot_three_dee_box(box, ax=ax)

plt.show()
