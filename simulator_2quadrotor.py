import numpy as np
from matplotlib import pyplot as plt
from matplotlib import animation
from model.quadrotor import Quadrotor
from Obstacle import plot_three_dee_box
from RRT_3D.RRT_star import RRT_star
from scipy.spatial.transform import Rotation
from simulator_helpers import generate_env, plot_all
from traj_optimization.cubic_spline import cubic_spline
from traj_optimization.mini_snap_optim import min_snap_optimizer_3d
from model.MPC_3D_dynamical_obstacle import MPC
#################################################################
# Create the quadrotor class and controller
env = Quadrotor()
env2 = Quadrotor()
policy = MPC()
policy2 = MPC()

# Define some initial values and state
dt = 0.01
t = 0
time_step = 1e-2
total_SE = 0
total_SE2 = 0
total_energy = 0
total_energy2 = 0
penalty = 2500
#################################################################
# Define the obstacles, plotting figure and axis and other scenario properties
scenario = 2
obstacles, fig, ax1, map_boundary, starts, ends = generate_env(scenario)
#########################################################################
# global path planning using RRT*
x_start = starts[0]
x_start2 = starts[1]
x_goal = ends[0]
x_goal2 = ends[1]

# RRT = RRT_star(x_start, 1500, obstacles, ax1, 1)
# RRT2 = RRT_star(x_start2, 1500, obstacles, ax1, 1)
# path_exists = RRT.find_path(x_goal, map_boundary)
# path_exists2 = RRT2.find_path(x_goal2, map_boundary)
path_exists = True
path_exists2 = True
#########################################################################
# Reset the quadrotor objects to the initial positions
current_state = env.reset(position=x_start)
current_state2 = env2.reset(position=x_start2)

# If a path has been found proceed to follow it
if not path_exists or not path_exists2:
    print("No path was found for the given number of iterations for the quadrotors")
else:
    print("Path found, applying smoothing.")
    # path_list = RRT.get_path()
    # path_list2 = RRT2.get_path()
    T = 25
    #pos, vel, acc = cubic_spline(path_list, T)
    #pos2, vel2, acc2 = cubic_spline(path_list2, T)
    print("Smoothing completed, tracking trajectory")
    # pos, vel, acc, jerk, snap, ts = min_snap_optimizer_3d(path_list, penalty, time_optimal=True)
    # pos2, vel2, acc2, jerk2, snap2, ts2 = min_snap_optimizer_3d(path_list2, penalty, time_optimal=True)
    # # pos, vel, acc, ts = min_snap_optimizer_3d(path_list)
    # np.savez('traj.npz', pos=pos, vel=vel, pos2=pos2, vel2=vel2, path_list=path_list, path_list2=path_list2)
    traj = np.load('traj.npz')
    pos = traj['pos']
    vel = traj['vel']
    pos2 = traj['pos2']
    vel2 = traj['vel2']
    path_list = traj['path_list']
    path_list2 = traj['path_list2']
    # ax1.plot(pos[:, 0], pos[:, 1], pos[:, 2], c='g', linewidth=2)
    # ax1.plot(pos2[:, 0], pos2[:, 1], pos2[:, 2], c='g', linewidth=2)
    real_trajectory = np.zeros((1, 3))
    real_orientation = np.zeros((1, 4))
    real_trajectory2 = np.zeros((1, 3))
    real_orientation2 = np.zeros((1, 4))
    # follow the path in segments
    for i in range(min(len(pos)-50, len(pos2-50))):
        # state_des = np.hstack((pos[i + 50], vel[i + 50], current_state2['x']))
        # state_des2 = np.hstack((pos2[i + 50], vel2[i + 50], current_state['x']))
        state_des = np.hstack((pos[i + 50], vel[i + 50], np.array([100, 100, 100])))
        state_des2 = np.hstack((pos2[i + 50], vel2[i + 50], np.array([100, 100, 100])))
        action = policy.control(current_state, state_des)
        action2 = policy2.control(current_state2, state_des2)
        cmd_rotor_speeds = action['cmd_rotor_speeds']
        cmd_rotor_speeds2 = action2['cmd_rotor_speeds']
        obs, reward, done, info = env.step(cmd_rotor_speeds)
        obs2, reward2, done2, info2 = env2.step(cmd_rotor_speeds2)
        print("current:", obs['x'])
        print('des_position: ', state_des[:3])
        print("quadrotor2_current:", obs2['x'])
        print('quadrotor2_des_position: ', state_des2[:3])
        if i == 0:
            real_trajectory = np.reshape(obs['x'], (1, 3))
            real_orientation = np.reshape(obs['q'], (1, 4))
            real_trajectory2 = np.reshape(obs2['x'], (1, 3))
            real_orientation2 = np.reshape(obs2['q'], (1, 4))
        else:
            real_trajectory = np.vstack((real_trajectory, np.reshape(obs['x'], (1, 3))))
            real_orientation = np.vstack((real_orientation, np.reshape(obs['q'], (1, 4))))
            real_trajectory2 = np.vstack((real_trajectory2, np.reshape(obs2['x'], (1, 3))))
            real_orientation2 = np.vstack((real_orientation2, np.reshape(obs2['q'], (1, 4))))
        current_state = obs
        current_state2 = obs2
        t += dt
        total_SE += (np.sum((obs['x'] - state_des[:3]) ** 2) * time_step)
        total_energy += (np.sum(cmd_rotor_speeds ** 2) * time_step)
        total_SE2 += (np.sum((obs2['x'] - state_des2[:3]) ** 2) * time_step)
        total_energy2 += (np.sum(cmd_rotor_speeds2 ** 2) * time_step)

    ############################################################################
    print("Sum of tracking error (integration): ", total_SE)
    print("Total time: ", t)
    print("Sum of energy consumption (integration)", total_energy)
    ############################################################################
    print("##########################################################")
    print("Quadrotor2:")
    print("Sum of tracking error (integration): ", total_SE)
    print("Total time: ", t)
    print("Sum of energy consumption (integration)", total_energy)
    ############################################################################

    # Plot the obstacles
    for box in obstacles:
        plot_three_dee_box(box, ax=ax1)
    # Plot the start and goal points
    ax1.plot([x_start[0]], [x_start[1]], [x_start[2]], marker='o', c='r', markersize=10)
    ax1.plot([x_goal[0]], [x_goal[1]], [x_goal[2]], marker='o', c='b', markersize=10)
    ax1.plot([x_start2[0]], [x_start2[1]], [x_start2[2]], marker='o', c='r', markersize=10)
    ax1.plot([x_goal2[0]], [x_goal2[1]], [x_goal2[2]], marker='o', c='b', markersize=10)

    # # Plot the final path
    # path_length = 0
    # for i in range(len(path_list) - 1):
    #     ax1.plot([path_list[i][0], path_list[i + 1][0]],
    #             [path_list[i][1], path_list[i + 1][1]],
    #             [path_list[i][2], path_list[i + 1][2]], c='b', linewidth=2)
    #     path_length += np.linalg.norm(path_list[i] - path_list[i + 1])
    # print('Length of path:', round(path_length, 2))
    #
    # # Plot the final path of quadrotor 2
    # for i in range(len(path_list2) - 1):
    #     ax1.plot([path_list2[i][0], path_list2[i + 1][0]],
    #             [path_list2[i][1], path_list2[i + 1][1]],
    #             [path_list2[i][2], path_list2[i + 1][2]], c='b', linewidth=2)

    # Plot the trajectory of the quadrotor
    rot = Rotation.from_quat(real_orientation[0, :]).as_matrix()
    print(rot)
    rotor_dists = np.array([[0.046*np.sqrt(2), -0.046*np.sqrt(2), 0],
                            [0.046*np.sqrt(2), 0.046*np.sqrt(2), 0],
                            [-0.046*np.sqrt(2), 0.046*np.sqrt(2), 0],
                            [-0.046*np.sqrt(2), -0.046*np.sqrt(2), 0]])
    rots = rotor_dists @ rot
    rots = rots + real_trajectory[0, :]
    points = np.vstack((real_trajectory[0, :], rots))
    point, = ax1.plot(points[:, 0], points[:, 1], points[:, 2], 'r.', label='Quadrotor')
    line, = ax1.plot([real_trajectory[0, 0]], [real_trajectory[0, 1]], [real_trajectory[0, 2]], 'r',
                     label='Real_Trajectory')
########################################################################################################
    # Plot the trajectory of the quadrotor2
    rot2 = Rotation.from_quat(real_orientation2[0, :]).as_matrix()
    print(rot2)
    rots2 = rotor_dists @ rot2
    rots2 = rots2 + real_trajectory2[0, :]
    points2 = np.vstack((real_trajectory2[0, :], rots2))
    point2, = ax1.plot(points2[:, 0], points2[:, 1], points2[:, 2], 'r.', label='Quadrotor2')
    line2, = ax1.plot([real_trajectory2[0, 0]], [real_trajectory2[0, 1]], [real_trajectory2[0, 2]], 'g',
                     label='Real_Trajectory2')

    # Helper function for the animation
    def animate(i):
        line.set_xdata(real_trajectory[:i + 1, 0])
        line.set_ydata(real_trajectory[:i + 1, 1])
        line.set_3d_properties(real_trajectory[:i + 1, 2])
        rot = Rotation.from_quat(real_orientation[i, :]).as_matrix()
        rots = rotor_dists @ rot
        rots = rots + real_trajectory[i, :]
        points = np.vstack((real_trajectory[i, :], rots))
        point.set_xdata(points[:, 0])
        point.set_ydata(points[:, 1])
        point.set_3d_properties(points[:, 2])
    def animate2(i):
        line.set_xdata(real_trajectory2[:i + 1, 0])
        line.set_ydata(real_trajectory2[:i + 1, 1])
        line.set_3d_properties(real_trajectory2[:i + 1, 2])
        rot = Rotation.from_quat(real_orientation2[i, :]).as_matrix()
        rots = rotor_dists @ rot
        rots = rots + real_trajectory2[i, :]
        points2 = np.vstack((real_trajectory2[i, :], rots))
        point.set_xdata(points2[:, 0])
        point.set_ydata(points2[:, 1])
        point.set_3d_properties(points2[:, 2])

    ax1.legend(loc='lower right')
    ani = animation.FuncAnimation(fig=fig, func=animate, frames=np.size(real_trajectory,0), interval=1, repeat=False,
                                  blit=False)
    # ani = animation.FuncAnimation(fig=fig, func=animate2, frames=np.size(real_trajectory2,0), interval=1, repeat=False,
    #                               blit=False)

    # RRT.plotTree()
    plt.show()
