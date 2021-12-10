import numpy as np
from math import factorial
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.optimize import minimize


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


def obj_function(variables):
    ts = variables[:n_seg] 
    xs = variables[n_seg:]

    qs = get_q(n_seg, n_order, ts)
    obj = xs @ qs @ xs.reshape(-1,1) + penalty*np.sum(ts**2)

    return obj


def equal_constraint(variables):
    ts = variables[:n_seg] 
    xs = variables[n_seg:]

    start_cond = np.array([waypoints[0], 0, 0, 0])
    end_cond = np.array([waypoints[-1], 0, 0, 0])
    aeq, beq = get_ab(n_seg, n_order, waypoints, ts, start_cond, end_cond)

    return aeq@xs-beq


def bound(n_seg, n_order):
    nvar = n_seg * (n_order + 1) + n_seg

    # set the boundary of the time distribution
    bound_tuple = ()
    for i in range(nvar):
        if i < n_seg:
            bound_tuple += ((0.5,2),)
        else:
            bound_tuple += ((-np.inf, np.inf),)
    return bound_tuple


def minimum_snap_np(waypoints, n_seg, n_order):
    nvar = n_seg * (n_order + 1) + n_seg

    # initial guess
    x0 = np.zeros(nvar)
    x0[:n_seg] = 1

    con1 = {'type': 'eq', 'fun': equal_constraint}
    con = ([con1])
    bnds = bound(n_seg, n_order)

    solution = minimize(obj_function, x0, method='SLSQP', bounds=bnds, constraints=con)

    return solution.x


def min_snap_optimizer_3d(path):
    n_order = 7
    n_seg = np.size(path, axis=0) - 1
    
    # this will introduce three optimization problems, ts are not independent
    ts = minimum_snap_np(path[:, 0], n_seg, n_order)[:n_seg]
    pos_coef_x = minimum_snap_np(path[:, 0], n_seg, n_order)[n_seg:]
    # pos_coef_y = minimum_snap_np(path[:, 1], n_seg, n_order)
    # pos_coef_z = minimum_snap_np(path[:, 2], n_seg, n_order)

    vel_coef_x = (pos_coef_x.reshape((-1, n_order+1))[:, 1:] * np.arange(start=1, stop=n_order+1).reshape(1, -1)).reshape((-1,))
    # vel_coef_y = (pos_coef_y.reshape((-1, n_order+1))[:, 1:] * np.arange(start=1, stop=n_order+1).reshape(1, -1)).reshape((-1,))
    # vel_coef_z = (pos_coef_z.reshape((-1, n_order+1))[:, 1:] * np.arange(start=1, stop=n_order+1).reshape(1, -1)).reshape((-1,))

    acc_coef_x = (vel_coef_x.reshape((-1, n_order))[:, 1:] * np.arange(start=1, stop=n_order).reshape(1, -1)).reshape((-1,))
    # acc_coef_y = (vel_coef_y.reshape((-1, n_order))[:, 1:] * np.arange(start=1, stop=n_order).reshape(1, -1)).reshape((-1,))
    # acc_coef_z = (vel_coef_z.reshape((-1, n_order))[:, 1:] * np.arange(start=1, stop=n_order).reshape(1, -1)).reshape((-1,))

    pos, vel, acc = [], [], []
    tstep = 0.01

    for i in range(n_seg):
        pxi = pos_coef_x[(8 * i):(8 * (i + 1))].tolist()
        # pyi = pos_coef_y[(8 * i):(8 * (i + 1))].tolist()
        # pzi = pos_coef_z[(8 * i):(8 * (i + 1))].tolist()

        vxi = vel_coef_x[(7 * i):(7 * (i + 1))].tolist()
        # vyi = vel_coef_y[(7 * i):(7 * (i + 1))].tolist()
        # vzi = vel_coef_z[(7 * i):(7 * (i + 1))].tolist()

        axi = acc_coef_x[(6 * i):(6 * (i + 1))].tolist()
        # ayi = acc_coef_y[(6 * i):(6 * (i + 1))].tolist()
        # azi = acc_coef_z[(6 * i):(6 * (i + 1))].tolist()

        for t in np.arange(0, ts[i], tstep):
            pos.append(np.polyval(pxi[::-1], t))
            # pos.append(np.polyval(pyi[::-1], t))
            # pos.append(np.polyval(pzi[::-1], t))

            vel.append(np.polyval(vxi[::-1], t))
            # vel.append(np.polyval(vyi[::-1], t))
            # vel.append(np.polyval(vzi[::-1], t))

            acc.append(np.polyval(axi[::-1], t))
            # acc.append(np.polyval(ayi[::-1], t))
            # acc.append(np.polyval(azi[::-1], t))

    # pos = np.array(pos).reshape((-1, 3))
    # vel = np.array(vel).reshape((-1, 3))
    # acc = np.array(acc).reshape((-1, 3))
    pos = np.array(pos).reshape((-1, 1))
    vel = np.array(vel).reshape((-1, 1))
    acc = np.array(acc).reshape((-1, 1))
    return pos, vel, acc, ts


path_list = np.array([[ 5.        ,  7.        ,  3.        ],
                      [10.86112711,  6.12592946,  2.55994633],
                      [13.91474856,  5.87165097,  2.46509981],
                      [14.26590969,  4.75821649,  2.06223386],
                      [12.38218297,  3.92085155,  2.11475727],
                      [10.97662532,  3.23206355,  1.86315257],
                      [ 9.50077495,  1.94620373,  1.79910651],
                      [ 7.62894844,  1.24818373,  1.74496482],
                      [ 7.18167236,  1.23754264,  1.59337815],
                      [ 5.49010816,  0.97186878,  1.41324015],
                      [ 4.66375168,  0.55217968,  1.62723241],
                      [ 3.35235155,  0.62747605,  1.70443546],
                      [ 1.64982418,  1.60245634,  2.04395953],
                      [ 0.5       ,  2.5       ,  1.5       ]])

# global variable
n_seg = path_list.shape[0]-1
penalty = 1
n_order = 7
waypoints = path_list[:,0]

# compute the optimal path
pos, vel, acc, ts = min_snap_optimizer_3d(path_list)
print('Time distribution:\n',np.round(ts,2))

N = len(vel[:,0])
fig, axs = plt.subplots(3)
fig.suptitle('Time optimal distributed')

axs[0].plot(range(N), pos[:,0])
# axs[0].plot(range(N), pos[:,1])
# axs[0].plot(range(N), pos[:,2])
axs[0].set_title('pos')

axs[1].plot(range(N), vel[:,0])
# axs[1].plot(range(N), vel[:,1])
# axs[1].plot(range(N), vel[:,2])
axs[1].set_title('vel')

axs[2].plot(range(N), acc[:,0])
# axs[2].plot(range(N), acc[:,1])
# axs[2].plot(range(N), acc[:,2])
axs[2].set_title('acc')
plt.show()