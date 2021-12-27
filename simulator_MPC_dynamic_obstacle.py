import numpy as np
from RRT_3D.RRT_star_plotter import RRT_star
from simulator_helpers import generate_env, plot_all, init_simulation
from traj_optimization.cubic_spline import cubic_spline
from traj_optimization.mini_snap_optim import min_snap_optimizer_3d

if __name__ == "__main__":
    # Create the quadrotor class, controller and other initial values
    env, policy, t, time_step, total_SE, total_energy, penalty = init_simulation(mpc=True, dynamic=True, time_horizon = 50)
    #################################################################
    # Define the obstacles, plotting figure and axis and other scenario properties
    scenario = 0
    obstacles, fig, ax1, map_boundary, starts, ends = generate_env(scenario)
    #########################################################################
    # global path planning using RRT*
    x_start = starts[0]
    x_goal = ends[0]

    # RRT = RRT_star(x_start, 1500, obstacles, 1)
    # path_exists = RRT.find_path(x_goal, map_boundary)
    path_exists = True
    #########################################################################

    current_state = env.reset(position=x_start)

    # If a path has been found, proceed to follow it
    if not path_exists:
        print("No path was found for the given number of iterations")
    else:
        print("Path found, applying smoothing.")
        # path_list = RRT.get_path()
        # pos, vel, acc = cubic_spline(path_list, T=25)
        print("Smoothing completed, tracking trajectory")
        # load the pre-saved trajectory
        traj = np.load('traj.npz')
        path_list = traj['path_list']
        pos = traj['pos']
        vel = traj['vel']
        obstacle_traj = np.flipud(pos) # reverse the trajectory as obstacle trajectory
        # pos, vel, acc, jerk, snap, ts = min_snap_optimizer_3d(path_list, penalty, time_optimal=True)
        ax1.plot(pos[:, 0], pos[:, 1], pos[:, 2], c='g', linewidth=2)
        real_trajectory = np.zeros((1, 3))
        real_orientation = np.zeros((1, 4))
        # follow the path in segments
        for i in range(len(pos)-policy.model.N):
            # static obstacle
            # show_up_time = int(0.5 * len(pos))
            # pos_obstacle = pos[show_up_time]

            # dynamic obstacle
            pos_obstacle = obstacle_traj[i]

            print("obstacle position: ", pos_obstacle)
            # if the agent is close to the obstacle, then avoid it
            if np.sum((current_state['x'] - pos_obstacle)**2) <= 1.5:
                state_des = np.hstack((pos[i + 4*policy.model.N], vel[i + 4*policy.model.N], pos_obstacle))
                print("avoiding obstacle......")
            else:
                state_des = np.hstack((pos[i + policy.model.N], vel[i + policy.model.N], np.array([100, 100, 100])))
            # state_des = np.hstack((pos[i + policy.model.N], vel[i + policy.model.N], pos_obstacle))
            # state_des = np.hstack((pos[i + 50], vel[i + 50], np.array([100, 100, 100])))
            action = policy.control(current_state, state_des)
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

        ############################################################################
        print("Sum of tracking error (integration): ", total_SE)
        print("Total time: ", t)
        print("Sum of energy consumption (integration)", total_energy)
        ############################################################################

        plot_all(fig, ax1, obstacles, x_start, x_goal, path_list, real_trajectory, real_orientation, dynamic=True, obstacle_trajectory = obstacle_traj)


