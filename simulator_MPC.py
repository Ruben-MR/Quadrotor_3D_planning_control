'''
File for simulation of the quadrotor in a given environment. This file is used for tracking:
    - A path generated by RRT_star
    - Converted into a trajectory by means of cubic spline interpolation or minimum snap
    - Tracked by means of a model-predictive controller (MPC)
'''

import numpy as np
from RRT_3D.RRT_star_plotter import RRT_star
from simulator_helpers import generate_env, plot_all, init_simulation, find_closest
from traj_optimization.mini_snap_optim import min_snap_optimizer_3d
from scipy import interpolate

if __name__ == "__main__":
    '''
    Some parameters for different performance/scenario settings
    '''
    ################################################################
    # some parameters
    obstacle = True                         # whether to have a local obstacle
    RRT_iterations = 2000                   # Number of iterations of the RRT_star algorithm
    use_pre_saved_traj = False              # whether to generate new trajectory using RRT* + trajectory smoothing
    collision_avoidance_guarantee = True    # whether to provide collision avoidance guarantee
    waypoint_navigation = False             # whether to use waypoints navigation instead of min_snap trajectory
    slow_factor = 1.8                       # whether to slow down the time-optimal trajectory
    scenario = 2                            # which scenario to test the algorithm on
    ###############################################################
    # Create the quadrotor class, controller and other initial values
    env, policy, t, time_step, total_SE, total_energy, penalty = init_simulation(mpc=True, time_horizon=40, obstacle=obstacle)
    #################################################################
    # Define the obstacles, plotting figure and axis and other scenario properties
    obstacles, fig, ax1, map_boundary, starts, ends = generate_env(scenario)
    #########################################################################
    # global path planning using RRT*
    x_start = starts[0]
    x_goal = ends[0]

    if not use_pre_saved_traj:
        RRT = RRT_star(x_start, RRT_iterations, obstacles, margin=0.5)
        path_exists = RRT.find_path(x_goal, map_boundary)
    else:
        path_exists = True
    #########################################################################

    current_state = env.reset(position=x_start)
    num_collision = 0

    # If a path has been found, proceed to follow it
    if not path_exists:
        print("No path was found for the given number of iterations")
    else:
        print("Path found, applying smoothing.")

        if not use_pre_saved_traj:
            # Apply the path interpolation algorithm selected, the path is simplified to make if time-feasible
            path_list = RRT.get_straight_path()
            # Parameters can be adjusted as necessary
            pos, vel, acc, jerk, snap, ts = min_snap_optimizer_3d(path_list, penalty=penalty, time_optimal=True,
                                                                  act_const=False, check_collision=True,
                                                                  obstacles=obstacles)
            # save the trajectory
            np.savez('traj.npz', pos=pos, vel=vel, acc=acc, path_list=path_list)
        else:
            # load the pre-saved trajectory
            traj = np.load('traj.npz')
            path_list = traj['path_list']
            pos = traj['pos']
            vel = traj['vel']

        print("Smoothing completed, tracking trajectory")

        # Plot the initial point (may don't need it)
        ax1.plot(pos[:, 0], pos[:, 1], pos[:, 2], c='mediumorchid', linewidth=2, label='Planned_path')
        real_trajectory = np.zeros((1, 3))
        real_orientation = np.zeros((1, 4))

        if not waypoint_navigation:
            # interpolation of time-optimal trajectory
            x = np.linspace(0, len(pos)-1, len(pos))
            f_pos = interpolate.interp1d(x, pos, axis=0)
            f_vel = interpolate.interp1d(x, vel, axis=0)
            x_interp = np.linspace(0, len(pos)-1, int(slow_factor * len(pos)))
            pos = f_pos(x_interp)
            vel = f_vel(x_interp) / slow_factor

            # follow the path in segments
            for i in range(len(pos) - policy.model.N):
                if obstacle:
                    # static obstacle
                    show_up_time = int(0.55 * len(pos))
                    pos_obstacle = pos[show_up_time]

                if obstacle:
                    # if the agent is close to the obstacle, then avoid it
                    if np.sum((current_state['x'] - pos_obstacle)**2) <= 4:
                        state_des = np.hstack((pos[i + 2*policy.model.N], vel[i + 2*policy.model.N], pos_obstacle))
                        print("avoiding obstacle......")
                    else:
                        state_des = np.hstack((pos[i + policy.model.N], vel[i + policy.model.N], np.array([100, 100, 100])))
                else:
                    state_des = np.hstack((pos[i + policy.model.N], vel[i + policy.model.N]))

                closest_obstacle = find_closest(current_state, obstacles)
                if closest_obstacle <= 0:
                    num_collision += 1
                bbox_size = closest_obstacle / np.sqrt(3)
                print("closest obstacle: ", bbox_size)
                if collision_avoidance_guarantee:
                    action = policy.control(current_state, state_des, bounding_box_size=bbox_size)
                else:
                    action = policy.control(current_state, state_des, bounding_box_size=50)
                cmd_rotor_speeds = action['cmd_rotor_speeds']
                obs, reward, done, info = env.step(cmd_rotor_speeds)
                print("current:", obs['x'])
                print('des_position: ', state_des[:3])
                if i == 0:
                    real_trajectory = np.reshape(obs['x'], (1, 3))
                    real_orientation = np.reshape(obs['q'], (1, 4))
                else:
                    real_trajectory = np.vstack((real_trajectory, np.reshape(obs['x'], (1, 3))))
                    real_orientation = np.vstack((real_orientation, np.reshape(obs['q'], (1, 4))))
                current_state = obs
                t += time_step
                total_SE += (np.sum((obs['x'] - state_des[:3]) ** 2) * time_step)
                total_energy += (np.sum(cmd_rotor_speeds ** 2) * time_step)
                # if current position is very close to the goal point, terminate the loop
                if np.sqrt(np.sum((obs['x'] - pos[-1]) ** 2)) <= 0.1:
                    break
        # use waypoint navigation, no velocity information, only waypoints given by RRT*
        else:
            down_sampling_points = 80
            waypoints_idx = np.linspace(0, len(pos)-1, down_sampling_points).astype(np.int)
            waypoints = pos[waypoints_idx]
            waypoints_vel = vel[waypoints_idx]/5
            # waypoints = path_list
            i = 0

            while(True):
                if obstacle:
                    # static obstacle
                    show_up_time = int(0.5 * len(pos))
                    pos_obstacle = pos[show_up_time]
                    print("obstacle position: ", pos_obstacle)

                if obstacle:
                    # if the agent is close to the obstacle, then avoid it
                    if np.sum((current_state['x'] - pos_obstacle)**2) <= 4:
                        state_des = np.hstack((waypoints[i], np.zeros(3), pos_obstacle))
                        print("avoiding obstacle......")
                    else:
                        state_des = np.hstack((waypoints[i], waypoints_vel[i], np.array([100, 100, 100])))
                else:
                    state_des = np.hstack((waypoints[i], waypoints_vel[i]))

                closest_obstacle = find_closest(current_state, obstacles)
                if closest_obstacle <= 0:
                    num_collision += 1
                bbox_size = closest_obstacle / np.sqrt(3)
                print("closest obstacle: ", bbox_size)

                if collision_avoidance_guarantee:
                    action = policy.control(current_state, state_des, bounding_box_size=bbox_size)
                else:
                    action = policy.control(current_state, state_des, bounding_box_size=50)

                cmd_rotor_speeds = action['cmd_rotor_speeds']
                obs, reward, done, info = env.step(cmd_rotor_speeds)
                print("current:", obs['x'])
                print('des_position: ', state_des[:3])
                if i == 0:
                    real_trajectory = np.reshape(obs['x'], (1, 3))
                    real_orientation = np.reshape(obs['q'], (1, 4))
                else:
                    real_trajectory = np.vstack((real_trajectory, np.reshape(obs['x'], (1, 3))))
                    real_orientation = np.vstack((real_orientation, np.reshape(obs['q'], (1, 4))))
                current_state = obs
                t += time_step
                total_SE += (np.sum((obs['x'] - state_des[:3]) ** 2) * time_step)
                total_energy += (np.sum(cmd_rotor_speeds ** 2) * time_step)
                # if current position is close to the current waypoint, give it the next one
                if np.sqrt(np.sum((obs['x'] - waypoints[i]) ** 2)) <= 0.3 and i < len(waypoints)-1:
                    i += 1
                # if current position is very close to the goal point, terminate the loop
                if np.sqrt(np.sum((obs['x'] - waypoints[-1]) ** 2)) <= 0.5 and i == len(waypoints)-1:
                    break
                elif t >= 80:
                    break

        # Print the final metrics of the simulation
        print("Sum of tracking error (integration): ", total_SE)
        print("Total time: ", t)
        print("Sum of energy consumption (integration)", total_energy)
        print("Number of collisions: ", num_collision)

        if obstacle:
            ax1.plot(pos_obstacle[0], pos_obstacle[1], pos_obstacle[2], marker='o', c='y', markersize=16)

        plot_all(fig, ax1, obstacles, x_start, x_goal, path_list, real_trajectory, real_orientation)

        print(len(pos))