import numpy as np
from math import factorial

########################################################################################################################
# FUNCTIONS FOR MINIMUM SNAP DEFINITION (BOTH FOR OPTIMAL TIME ALLOCATION AND NORMAL)
########################################################################################################################


# Get the matrix in objective function
def get_q(n_seg, n_order, ts):
    q = np.zeros((n_seg * (n_order + 1), n_seg * (n_order + 1)))
    for k in range(n_seg):
        q_k = np.zeros((n_order + 1, n_order + 1))

        # Get the Q matrix for each segment of the trajectory
        for i in range(4, n_order + 1):
            for l in range(4, n_order + 1):
                q_k[i, l] = i*(i-1)*(i-2)*(i-3)*l*(l-1)*(l-2)*(l-3)*pow(ts[k], (i+l-7)) / (i+l-7)
        # Assign it to the corresponding pos of the general Q matrix
        q[(8 * k):(8 * k + 8), (8 * k):(8 * k + 8)] = q_k
    return q


# Get the constraint matrices
def get_ab(n_seg, n_order, waypoints, ts):
    n_all_poly = n_seg * (n_order + 1)
    # set initial and final point constraints
    aeq_start = np.zeros((4, n_all_poly))
    aeq_start[:4, :4] = np.diag([1, 1, 2, 6])
    aeq_end = np.zeros((4, n_all_poly))
    for k in range(4):
        for i in range(k, n_order + 1):
            aeq_end[k, -(n_order + 1 - i)] = (factorial(i) * pow(ts[-1], (i - k))) / factorial(i - k)
    beq_start = np.array([waypoints[0], 0, 0, 0])
    beq_end = np.array([waypoints[-1], 0, 0, 0])

    # pos constraints for waypoints
    aeq_wp = np.zeros((n_seg - 1, n_all_poly))
    for j in range(n_seg - 1):
        aeq_wp[j, 8 * (j + 1)] = 1
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
    beq_cont = np.zeros(4 * (n_seg - 1), )
    aeq = np.vstack((aeq_start, aeq_end, aeq_wp, aeq_cont))
    beq = np.concatenate((beq_start, beq_end, beq_wp, beq_cont))
    return aeq, beq


# Get the continuity constraint matrix
def get_aeq_cont(n_seg, n_order, ts, k):
    aeq_cont = np.zeros((n_seg - 1, n_seg * (n_order + 1)))
    for j in range(n_seg - 1):
        for i in range(k, n_order + 1):
            aeq_cont[j, (n_order + 1) * j + i] = factorial(i) * pow(ts[j], i - k) / factorial(i - k)
        aeq_cont[j, (n_order + 1) * (j + 1) + k] = -factorial(k)
    return aeq_cont

########################################################################################################################
# FUNCTIONS SPECIFIC FOR OPTIMAL TIME ALLOCATION
########################################################################################################################


# Variable bounds for time-allocation minimum snap
def bound(n_seg, n_var):
    # set the boundary of the time distribution
    bound_tuple = ()
    for i in range(n_var):
        if i < n_seg:
            bound_tuple += ((0.5, 2),)
        else:
            bound_tuple += ((-np.inf, np.inf),)
    return bound_tuple


# Objective function for time-allocation minimum snap
def obj_function(variables, n_seg, n_order, penalty):
    # Un-pack the variables
    ts = variables[:n_seg]
    xs = variables[n_seg: n_seg * (n_order + 1) + n_seg]
    ys = variables[n_seg * (n_order + 1) + n_seg: 2 * n_seg * (n_order + 1) + n_seg]
    zs = variables[2 * n_seg * (n_order + 1) + n_seg: 3 * n_seg * (n_order + 1) + n_seg]

    # Get the cost function matrix for the coefficients
    qs = get_q(n_seg, n_order, ts)
    obj = xs @ qs @ xs.reshape(-1, 1) + ys @ qs @ ys.reshape(-1, 1) + zs @ qs @ zs.reshape(-1, 1) \
          + penalty * np.sum(ts ** 2)

    return obj


# Equality constraint function for time-allocation minimum snap
def equal_constraint(variables, n_seg, n_order, path_list):
    # Unpack the different variables
    ts = variables[:n_seg]
    xs = variables[n_seg:n_seg * (n_order + 1) + n_seg]
    ys = variables[n_seg * (n_order + 1) + n_seg: 2 * n_seg * (n_order + 1) + n_seg]
    zs = variables[2 * n_seg * (n_order + 1) + n_seg: 3 * n_seg * (n_order + 1) + n_seg]

    # Obtain the different constraint matrices
    aeq_x, beq_x = get_ab(n_seg, n_order, path_list[:, 0], ts)
    aeq_y, beq_y = get_ab(n_seg, n_order, path_list[:, 1], ts)
    aeq_z, beq_z = get_ab(n_seg, n_order, path_list[:, 2], ts)

    # Compute the equality value
    constraint = np.hstack((aeq_x @ xs - beq_x,
                            aeq_y @ ys - beq_y,
                            aeq_z @ zs - beq_z))
    return constraint

########################################################################################################################
# FUNCTIONS SPECIFIC FOR PROPORTIONAL TIME ALLOCATION
########################################################################################################################

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

def obj_function_normal(variables, n_seg, n_order, ts):
    qs = get_q(n_seg, n_order, ts)
    obj = variables @ qs @ variables.reshape(-1, 1)
    return obj

def equal_constraint_normal(variables, n_seg, n_order, waypoints, ts):
    aeq, beq = get_ab(n_seg, n_order, waypoints, ts)
    constraint = aeq @ variables - beq
    return constraint