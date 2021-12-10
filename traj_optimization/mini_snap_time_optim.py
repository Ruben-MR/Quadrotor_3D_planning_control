import numpy as np
from math import factorial
from matplotlib import pyplot as plt
from scipy.optimize import minimize


# get the matrix in objective function
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

# get the continuity constraint matrix
def get_aeq_cont(n_seg, n_order, ts, k):
    aeq_cont = np.zeros((n_seg - 1, n_seg * (n_order + 1)))
    for j in range(n_seg - 1):
        for i in range(k, n_order + 1):
            aeq_cont[j, (n_order+1) * j + i] = factorial(i)*pow(ts[j], i-k)/factorial(i-k)
        aeq_cont[j, (n_order+1) * (j + 1) + k] = -factorial(k)
    return aeq_cont

# get the constraint matrix
def get_ab(n_seg, n_order, waypoints, ts):
    n_all_poly = n_seg * (n_order + 1)
    # set initial and final point constraints
    aeq_start = np.zeros((4, n_all_poly))
    aeq_start[:4, :4] = np.diag([1, 1, 2, 6])
    aeq_end = np.zeros((4, n_all_poly))
    for k in range(4):
        for i in range(k, n_order+1):
            aeq_end[k, -(n_order + 1 - i)] = (factorial(i)*pow(ts[-1], (i-k)))/factorial(i-k)
    beq_start = np.array([waypoints[0], 0, 0, 0])
    beq_end = np.array([waypoints[-1], 0, 0, 0])

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

# get the objective function
def obj_function(variables):
    ts = variables[:n_seg] 
    xs = variables[n_seg: n_seg*(n_order+1)+n_seg]
    ys = variables[n_seg*(n_order+1)+n_seg: 2*n_seg*(n_order+1)+n_seg]
    zs = variables[2*n_seg*(n_order+1)+n_seg: 3*n_seg*(n_order+1)+n_seg]
    
    qs = get_q(n_seg, n_order, ts)
    obj = xs @ qs @ xs.reshape(-1,1) +\
          ys @ qs @ ys.reshape(-1,1) +\
          zs @ qs @ zs.reshape(-1,1) +\
          penalty*np.sum(ts**2)

    return obj

# get the equality constraints
def equal_constraint(variables):
    ts = variables[:n_seg] 
    xs = variables[n_seg:n_seg*(n_order+1)+n_seg]
    ys = variables[n_seg*(n_order+1)+n_seg: 2*n_seg*(n_order+1)+n_seg]
    zs = variables[2*n_seg*(n_order+1)+n_seg: 3*n_seg*(n_order+1)+n_seg]

    aeq_x, beq_x = get_ab(n_seg, n_order, path_list[:,0], ts)
    aeq_y, beq_y = get_ab(n_seg, n_order, path_list[:,1], ts)
    aeq_z, beq_z = get_ab(n_seg, n_order, path_list[:,2], ts)
    
    constraint = np.hstack((aeq_x@xs-beq_x,
                            aeq_y@ys-beq_y,
                            aeq_z@zs-beq_z))
    
    return constraint

# get the bounds of the variables
def bound(n_seg, n_order):
    nvar = 3 * n_seg * (n_order + 1) + n_seg

    # set the boundary of the time distribution
    bound_tuple = ()
    for i in range(nvar):
        if i < n_seg:
            bound_tuple += ((0.5,2),)
        else:
            bound_tuple += ((-np.inf, np.inf),)
    return bound_tuple

# nonlinear programming based optimal time allocation minimal snap
def minimum_snap_np(path, n_seg, n_order):
    # variables for time, x, y, z
    nvar = 3 * n_seg * (n_order + 1) + n_seg      
    path = path
    
    # initial guess
    x0 = np.zeros(nvar)
    x0[:n_seg] = 1
    
    # set the constraints and bounds
    con1 = {'type': 'eq', 'fun': equal_constraint}
    con = ([con1])
    bnds = bound(n_seg, n_order)
    
    # solve the problem
    solution = minimize(obj_function, x0, method='SLSQP', bounds=bnds, constraints=con)
    
    # get the solution
    ts = solution.x[:n_seg]
    pos_coef_x = solution.x[n_seg:n_seg*(n_order+1)+n_seg]
    pos_coef_y = solution.x[n_seg*(n_order+1)+n_seg: 2*n_seg*(n_order+1)+n_seg]
    pos_coef_z = solution.x[2*n_seg*(n_order+1)+n_seg: 3*n_seg*(n_order+1)+n_seg]
    
    return ts, pos_coef_x, pos_coef_y, pos_coef_z

# trajectory optimization
def min_snap_optimizer_3d(path):
    n_order = 7
    n_seg = np.size(path, axis=0) - 1
    
    # this will introduce three optimization problems, ts are not independent
    ts, pos_coef_x, pos_coef_y, pos_coef_z = minimum_snap_np(path, n_seg, n_order)

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
    return pos, vel, acc, ts


# global variables
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
n_seg = path_list.shape[0]-1
penalty = 1
n_order = 7

# compute the optimal path
pos, vel, acc, ts = min_snap_optimizer_3d(path_list)
print('Time distribution:\n',np.round(ts,2))

# plot the results
N = len(vel[:,0])
fig, axs = plt.subplots(3)
fig.suptitle('Time optimal distributed')

axs[0].plot(range(N), pos[:,0])
axs[0].plot(range(N), pos[:,1])
axs[0].plot(range(N), pos[:,2])
axs[0].set_title('pos')

axs[1].plot(range(N), vel[:,0])
axs[1].plot(range(N), vel[:,1])
axs[1].plot(range(N), vel[:,2])
axs[1].set_title('vel')

axs[2].plot(range(N), acc[:,0])
axs[2].plot(range(N), acc[:,1])
axs[2].plot(range(N), acc[:,2])
axs[2].set_title('acc')
plt.show()