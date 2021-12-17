"""
File containing helper functions for frequently repeated actions in the simulator files for the different conditions
evaluated.
"""
import numpy as np
from model.quadrotor import Quadrotor
from Obstacle import Obstacle, plot_three_dee_box
import matplotlib.pyplot as plt
from matplotlib import animation
from scipy.spatial.transform import Rotation
from model.MPC_3D import MPC
#from model.MPC_3D_dynamical_obstacle import MPC as MPC_dyn
from model.MPC_3D_intermediate_set import MPC as MPC_dyn
from model.nonlinear_controller import GeometricController


# Initiate environment variables and create some of the required objects,
# will generate the required policy depending on whether MPC is wanted or not
def init_simulation(mpc=True, dynamic=True):
    env = Quadrotor()
    if mpc and dynamic:
        policy = MPC_dyn()
    elif mpc and not dynamic:
        policy = MPC()
    else:
        policy = GeometricController()
    t0, dt, total_se, total_energy, penalty = 0, 1e-2, 0, 0, 2500
    return env, policy, t0, dt, total_se, total_energy, penalty


# Function for generating the obstacles object for different scenarios in which the drone is to be tested
# and the figures and axes of the plotting
def generate_env(scenario):
    # Create the list with the coordinates of the bounding box
    # TODO: create a separate file and load this data from there
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

    # Convert the list of points in a list of obstacles
    obstacles = list()
    for box in boxes:
        obstacles.append(Obstacle(box[0, :], box[1, :]))

    # Define the figure and axis for plotting depending on the setup
    # TODO: again, depending on the scenario, additional information for plotting could be loaded
    fig = plt.figure()
    axis = fig.add_subplot(111, projection="3d")
    axis.set_xlim(0, 15)
    axis.set_ylim(-5, 10)
    axis.set_zlim(0, 15)
    plt.rcParams['figure.figsize'] = 16, 16
    axis.set_xlabel('x')
    axis.set_ylabel('y')
    axis.set_zlabel('z')
    axis.set_title('3D animate')
    axis.view_init(30, 35)

    # Set the map boundaries for point search in RRT_star
    # TODO: this could be done once again by loading from a file
    boundary = [15, 8, 3]

    return obstacles, fig, axis, boundary


# Function for doing the visualization of all the objects and elements in the scenario and simulation
def plot_all(fig, axis, obstacles, start, goal, path, trajectory, orientation):
    # Plot the obstacles
    for box in obstacles:
        plot_three_dee_box(box, ax=axis)
    # Plot the start and goal points
    axis.plot([start[0]], [start[1]], [start[2]], marker='o', c='r', markersize=10)
    axis.plot([goal[0]], [goal[1]], [goal[2]], marker='o', c='b', markersize=10)

    # Plot the final path
    path_length = 0
    for i in range(len(path) - 1):
        axis.plot([path[i][0], path[i + 1][0]],
                 [path[i][1], path[i + 1][1]],
                 [path[i][2], path[i + 1][2]], c='b', linewidth=2)
        path_length += np.linalg.norm(path[i] - path[i + 1])
    print('Length of path:', round(path_length, 2))

    # Plot the trajectory of the quadrotor
    rot = Rotation.from_quat(orientation[0, :]).as_matrix()
    print(rot)
    rotor_dists = np.array([[0.046 * np.sqrt(2), -0.046 * np.sqrt(2), 0],
                            [0.046 * np.sqrt(2), 0.046 * np.sqrt(2), 0],
                            [-0.046 * np.sqrt(2), 0.046 * np.sqrt(2), 0],
                            [-0.046 * np.sqrt(2), -0.046 * np.sqrt(2), 0]])
    rots = rotor_dists @ rot
    rots = rots + trajectory[0, :]
    points = np.vstack((trajectory[0, :], rots))
    point, = axis.plot(points[:, 0], points[:, 1], points[:, 2], 'r.', label='Quadrotor')
    line, = axis.plot([trajectory[0, 0]], [trajectory[0, 1]], [trajectory[0, 2]], 'g', label='Real_Trajectory')

    # Helper function for the animation
    def animate(i):
        line.set_xdata(trajectory[:i + 1, 0])
        line.set_ydata(trajectory[:i + 1, 1])
        line.set_3d_properties(trajectory[:i + 1, 2])
        rot = Rotation.from_quat(orientation[i, :]).as_matrix()
        rots = rotor_dists @ rot
        rots = rots + trajectory[i, :]
        points = np.vstack((trajectory[i, :], rots))
        point.set_xdata(points[:, 0])
        point.set_ydata(points[:, 1])
        point.set_3d_properties(points[:, 2])

    axis.legend(loc='lower right')
    ani = animation.FuncAnimation(fig=fig, func=animate, frames=np.size(trajectory, 0), interval=1, repeat=False,
                                  blit=False)
    plt.show()
