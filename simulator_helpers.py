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
from model.MPC_3D_static_obstacle import MPC_sta
from model.MPC_3D_dynamic_obstacle import MPC_dyn
from model.nonlinear_controller import GeometricController
import csv
import os


# Initiate environment variables and create some required objects,
# will generate the required policy depending on whether MPC is wanted or not
def init_simulation(mpc=True, dynamic=True, time_horizon = 50):
    env = Quadrotor()
    if mpc and dynamic:
        policy = MPC_dyn(time_horizon)
    elif mpc and not dynamic:
        policy = MPC_sta(time_horizon)
    else:
        policy = GeometricController()
    t0, dt, total_se, total_energy, penalty = 0, 1e-2, 0, 0, 2500
    return env, policy, t0, dt, total_se, total_energy, penalty


# Function for generating the obstacles object for different scenarios in which the drone is to be tested
# and the figures and axes of the plotting
def generate_env(scenario):
    # Scenario: type int, the selected scenario among those present in the scenario directory.
    # Create the list with the coordinates of the bounding box

    this_dir = os.path.dirname(os.path.abspath(__file__))
    filename = f"{this_dir}/scenarios/scenario_{scenario}.csv"
    print(f"Loading scenario from {filename}")

    file = open(filename)
    csvreader = csv.reader(file)
    for i in range(7):
        next(csvreader)
    rows = []
    for row in csvreader:
        rows.append(row)
    file.close()

    # Extract the obstacles
    boxes = list()

    for row in rows[16:]:
        box = np.array([[float(row[1]), float(row[2]), float(row[3])],
                        [float(row[5]), float(row[6]), float(row[7])]])
        boxes.append(box)

    # Convert the list of points in a list of obstacles
    obstacles = list()
    for box in boxes:
        obstacles.append(Obstacle(box[0, :], box[1, :]))

    # Define the figure and axis for plotting depending on the setup
    fig = plt.figure()
    axis = fig.add_subplot(111, projection="3d")
    axis.set_xlim(float(rows[0][1]), float(rows[0][2]))
    axis.set_ylim(float(rows[1][1]), float(rows[1][2]))
    axis.set_zlim(float(rows[2][1]), float(rows[2][2]))
    plt.rcParams['figure.figsize'] = float(rows[3][1]), float(rows[3][2])
    axis.set_xlabel(rows[4][1].lstrip())
    axis.set_ylabel(rows[5][1].lstrip())
    axis.set_zlabel(rows[6][1].lstrip())
    axis.set_title(rows[7][1].lstrip())
    axis.view_init(float(rows[8][1]), float(rows[8][2]))

    # Set the map boundaries for point search in RRT_star
    boundary = [float(rows[10][1]), float(rows[10][2]), float(rows[10][3])]

    # Set the initial and terminal positions

    start_points = []
    end_points = []

    for idx in range(len(rows[12])//4):
        start_point = np.array([float(rows[12][1+4*idx]), float(rows[12][2+4*idx]), float(rows[12][3+4*idx])])
        end_point = np.array([float(rows[13][1+4*idx]), float(rows[13][2+4*idx]), float(rows[13][3+4*idx])])
        start_points.append(start_point)
        end_points.append(end_point)

    print("Loaded scenario successfully.")

    # TODO: there might be a better way to implement the scenarios for multiple drones.

    return obstacles, fig, axis, boundary, start_points, end_points


# Function for doing the visualization of all the objects and elements in the scenario and simulation
def plot_all(fig, axis, obstacles, start, goal, path, trajectory, orientation, dynamic = False, obstacle_trajectory = None):
    if dynamic:
        obstacle_trajectory = obstacle_trajectory[:len(trajectory)]
    # Plot the obstacles
    for box in obstacles:
        plot_three_dee_box(box, ax=axis)
    # Plot the start and goal points
    axis.plot([start[0]], [start[1]], [start[2]], 'go', markersize=5, label="Start")
    axis.plot([goal[0]], [goal[1]], [goal[2]], 'bo', markersize=5, label="End")

    # Plot the final path
    path_length = 0
    for i in range(len(path) - 1):
        if i == 0:
            axis.plot([path[i][0], path[i + 1][0]],
                      [path[i][1], path[i + 1][1]],
                      [path[i][2], path[i + 1][2]], c='b', linewidth=1, label="RRT_path")
        else:
            axis.plot([path[i][0], path[i + 1][0]],
                 [path[i][1], path[i + 1][1]],
                 [path[i][2], path[i + 1][2]], c='b', linewidth=1)
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
    # if dynamic:
    #     point_obstacle, = axis.plot(obstacle_trajectory[:, 0], obstacle_trajectory[:, 1], obstacle_trajectory[:, 2], 'y.', label='Obstacle')
    line, = axis.plot([trajectory[0, 0]], [trajectory[0, 1]], [trajectory[0, 2]], 'g', label='Real_Trajectory')
    print(points)
    print(point)
    print(line)


    # Helper function for the animation
    def animate(i):
        line.set_xdata(trajectory[:i + 1, 0])
        line.set_ydata(trajectory[:i + 1, 1])
        line.set_3d_properties(trajectory[:i + 1, 2])
        rot = Rotation.from_quat(orientation[i, :]).as_matrix()
        rots = rotor_dists @ rot
        rots = rots + trajectory[i, :]
        if dynamic:
            points = np.vstack((trajectory[i, :], rots, obstacle_trajectory[i, :]))
        else:
            points = np.vstack((trajectory[i, :], rots))
        point.set_xdata(points[:, 0])
        point.set_ydata(points[:, 1])
        point.set_3d_properties(points[:, 2])



    axis.legend(loc='lower right')
    ani = animation.FuncAnimation(fig=fig, func=animate, frames=np.size(trajectory, 0), interval=1, repeat=False,
                                  blit=False)
    plt.show()
