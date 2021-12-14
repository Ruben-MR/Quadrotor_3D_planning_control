import numpy as np
import math
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
            aeq_end[k, -(n_order + 1 - i)] = (math.factorial(i) * pow(ts[-1], (i - k))) / math.factorial(i - k)
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
            aeq_cont[j, (n_order + 1) * j + i] = math.factorial(i) * pow(ts[j], i - k) / math.factorial(i - k)
        aeq_cont[j, (n_order + 1) * (j + 1) + k] = -math.factorial(k)
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
# FUNCTIONS FOR ACTUATION CONSTRAINT
########################################################################################################################


def get_input_from_ref(acc, jerk, snap):
    m, g = 0.03, 9.81
    # Total thrust obtained from accelerations
    total_thrust = m*np.sqrt(acc[0]**2 + acc[1]**2 + (acc[2] - g)**2)

    # Pitch and roll angles considering zero yaw
    pitch = math.atan(acc[0]/(acc[2] - g))
    roll = math.asin(m*acc[1]/total_thrust)

    # Get the body frame axes
    xc = np.array([1, 0, 0])
    yc = np.array([0, 1, 0])
    zc = np.array([0, 0, 1])

    alpha = acc - g*zc
    xb = np.cross(yc, alpha)/np.linalg.norm(np.cross(yc, alpha))
    yb = np.cross(alpha, xb)/np.linalg.norm(np.cross(alpha, xb))
    zb = np.cross(xb, yb)

    # Compute angular velocities
    c = np.dot(zb, alpha)
    wb = np.zeros((3,))
    wb[0] = - np.dot(yb, jerk)/c
    wb[1] = np.dot(xb, jerk)/c
    wb[2] = wb[1]*np.dot(yc, zb)/np.linalg.norm(np.cross(yc, zb))

    # Compute angular accelerations
    c_dot = np.dot(zb, jerk)
    wb_dot = np.zeros((3,))
    wb_dot[0] = -(np.dot(yb, snap) - 2*c_dot*wb[0] + c*wb[1]*wb[2])/c
    wb_dot[1] = (np.dot(xb, snap) - 2*c_dot*wb[1] + c*wb[0]*wb[2])/c
    wb_dot[2] = (-wb[0]*wb[1]*np.dot(yc, yb) - wb[0]*wb[2]*np.dot(yc, zb) + wb_dot[1]*np.dot(yc, zb)) / \
                np.linalg.norm(np.cross(yc, zb))

    # Compute the moments
    inertia_mat = np.diag(np.array([1.43e-5, 1.43e-5, 2.89e-5]))
    m = np.dot(inertia_mat, wb_dot) + np.cross(wb, np.dot(inertia_mat, wb))

    # Convert to rotor speeds
    k_drag, k_thrust = 7.8e-11, 2.3e-08
    k = k_drag/k_thrust
    to_inputs = np.array([[1, 1, 1, 1],
                             [0, 0.046, 0, -0.046],
                             [-0.046, 0, 0.046, 0],
                             [k, -k, k, -k]])
    rotor_forces = np.dot(np.linalg.inv(to_inputs), np.array([total_thrust, m[0], m[1], m[2]]))
    rotor_speeds = np.sqrt(rotor_forces/k_thrust)
    return rotor_speeds


