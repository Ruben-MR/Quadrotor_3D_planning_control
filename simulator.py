import os
import sys
##################################################################
# deal with the folders

url1 = os.path.join(os.getcwd(), "model/")
url2 = os.path.join(os.getcwd(), "no_longer_needed/traj_handles_ro47001/")
sys.path.append(url1)
sys.path.append(url2)
#################################################################
import numpy as np
from matplotlib import pyplot as plt
from matplotlib import animation
from model.quadrotor import Quadrotor
from model.nonlinear_controller import GeometricController
from Obstacle import Obstacle, plot_three_dee_box
from RRT_3D.RRT_star import RRT_star
from scipy.spatial.transform import Rotation
# from traj_optimization.cubic_spline import cubic_spline
from traj_optimization.mini_snap_optim import min_snap_optimizer_3d
#################################################################

# Create the quadrotor class and controller
env = Quadrotor()
policy = GeometricController()

# Define some initial values and state
dt = 0.01
t = 0
time_step = 1e-2
total_SE = 0
total_energy = 0
penalty = 0
#################################################################
# Define the obstacles

boxes = list()
boxes.append(np.array([[0, 5, 0], [14, 5.3, 3]]))
boxes.append(np.array([[14, 5, 0], [15, 5.3, 2]]))
boxes.append(np.array([[0, 4, 0], [1, 5, 1]]))
boxes.append(np.array([[0, 2, 0], [1, 3, 1]]))
boxes.append(np.array([[1.5, 4, 0], [2.5, 5, 1]]))
boxes.append(np.array([[5, 0, 2], [5.3, 5, 3]]))
boxes.append(np.array([[5, 1, 1], [5.3, 4, 2]]))
boxes.append(np.array([[5, 0, 0], [5.3, 4, 1]]))
boxes.append(np.array([[2, 2.5, 0], [5, 2.8, 3]]))

obstacles = list()
for box in boxes:
    obstacles.append(Obstacle(box[0, :], box[1, :]))

# Set up the plotting tools and elements

fig = plt.figure()
ax1 = fig.add_subplot(111, projection="3d")
ax1.set_xlim(0, 15)
ax1.set_ylim(-5, 10)
ax1.set_zlim(0, 15)
plt.rcParams['figure.figsize'] = 16, 16
ax1.set_xlabel('x')
ax1.set_ylabel('y')
ax1.set_zlabel('z')
ax1.set_title('3D animate')
ax1.view_init(30, 35)
#########################################################################
# global path planning using RRT*
x_start = np.array([5, 7, 3])
x_goal = np.array([0.5, 2.5, 1.5])
map_boundary = [15, 8, 3]

current_state = env.reset(position=x_start)
RRT = RRT_star(x_start, 1500, obstacles, ax1, 1)
path_exists = RRT.find_path(x_goal, map_boundary)
#########################################################################
# If a path has been found proceed to follow it
if not path_exists:
    print("No path was found for the given number of iterations")
else:
    print("Path found, applying smoothing.")
    path_list = RRT.get_path()
    # pos, vel, acc = cubic_spline(path_list, T)
    pos, vel, acc, ts = min_snap_optimizer_3d(path_list, penalty, time_optimal = True)
    print("Smoothing completed, tracking trajectory")
    ax1.plot(pos[:, 0], pos[:, 1], pos[:, 2], c='g', linewidth=2)
    real_trajectory = np.zeros((1, 3))
    real_orientation = np.zeros((1, 4))
    # follow the path in segments
    for i in range(len(pos)):
        state_des = {'pos': pos[i], 'vel': vel[i], 'acc': acc[i], 'yaw': 0, 'yaw_dot': 0}
        action = policy.control(state_des, current_state)
        cmd_rotor_speeds = action['cmd_rotor_speeds']
        obs, reward, done, info = env.step(cmd_rotor_speeds)
        print("current:", obs['x'])
        print('des_position: ', state_des['pos'])
        if i == 0:
            real_trajectory = np.reshape(obs['x'], (1, 3))
            real_orientation = np.reshape(obs['q'], (1, 4))
        else:
            real_trajectory = np.vstack((real_trajectory, np.reshape(obs['x'], (1, 3))))
            real_orientation = np.vstack((real_orientation, np.reshape(obs['q'], (1, 4))))
        current_state = obs
        t += dt
        total_SE += (np.sum((obs['x'] - state_des['pos']) ** 2) * time_step)
        total_energy += (np.sum(cmd_rotor_speeds ** 2) * time_step)

    ###########################################################################
    print("Sum of tracking error (integration): ", total_SE)
    print("Total time: ", t)
    print("Sum of energy consumption (integration)", total_energy)
    ###########################################################################

    # Plot the obstacles
    for box in obstacles:
        plot_three_dee_box(box, ax=ax1)
    # Plot the start and goal points
    ax1.plot([x_start[0]], [x_start[1]], [x_start[2]], marker='o', c='r', markersize=10)
    ax1.plot([x_goal[0]], [x_goal[1]], [x_goal[2]], marker='o', c='b', markersize=10)
    # Plot the final path
    path_length = 0
    for i in range(len(path_list) - 1):
        ax1.plot([path_list[i][0], path_list[i + 1][0]],
                [path_list[i][1], path_list[i + 1][1]],
                [path_list[i][2], path_list[i + 1][2]], c='b', linewidth=2)
        path_length += np.linalg.norm(path_list[i] - path_list[i + 1])
    print('Length of path:', round(path_length, 2))

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
    line, = ax1.plot([real_trajectory[0, 0]], [real_trajectory[0, 1]], [real_trajectory[0, 2]], 'g',
                     label='Real_Trajectory')

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


    ax1.legend(loc='lower right')
    ani = animation.FuncAnimation(fig=fig, func=animate, frames=np.size(real_trajectory,0), interval=1, repeat=False,
                                  blit=False)
    # RRT.plotTree()
    plt.show()
