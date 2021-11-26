import os
import sys

##################################################################
# deal with the folders

url1 = os.path.join(os.getcwd(), "model/")
url2 = os.path.join(os.getcwd(), "traj_handles_ro47001/")
sys.path.append(url1)
sys.path.append(url2)

#################################################################
import numpy as np
from matplotlib import pyplot as plt
from matplotlib import animation
from model.quadrotor import Quadrotor
from model.nonlinear_controller import GeometricController
from traj_handles_ro47001.tj_handle_circle import tj_circle as tj_handle
from traj_handles_ro47001.tj_handle_circle import get_T_circle
from box_plotter import plot_three_dee_box
from Obstacle import Obstacle
#################################################################

env = Quadrotor()
# circle trajectory has different initial position
policy = GeometricController()
T = get_T_circle()
current_state = env.reset(position=[5, 0, 0])
dt = 0.01
t = 0
time_step = 1e-2
iterations = int(T / time_step)
real_trajectory = {'x': [], 'y': [], 'z': []}
total_SE = 0
total_energy = 0

for itr in range(iterations):
    state_des = tj_handle(t)
    action = policy.control(state_des, current_state)
    cmd_rotor_speeds = action['cmd_rotor_speeds']
    obs, reward, done, info = env.step(cmd_rotor_speeds)
    print("--------------------------")
    print("current:", obs['x'])
    print('des_position: ', state_des['pos'])
    print('error: ', np.round(obs['x'] - state_des['pos'], 3))
    print("time: ", t)
    real_trajectory['x'].append(obs['x'][0])
    real_trajectory['y'].append(obs['x'][1])
    real_trajectory['z'].append(obs['x'][2])
    current_state = obs
    t += dt
    total_SE += (np.sum((obs['x'] - state_des['pos']) ** 2) * time_step)
    total_energy += (np.sum(cmd_rotor_speeds ** 2) * time_step)

############################################################################
print("Sum of tracking error (integration): ", total_SE)
print("Total time: ", t)
print("Sum of energy consumption (integration)", total_energy)
############################################################################
# plot nice maps!

# set the environment
# Display the whole process(can be commented all below when importing the search function)
boxes = list()
boxes.append(np.array([[0, 5, 0], [14, 5.3, 3]]))
boxes.append(np.array([[14, 5, 0], [15, 5.3, 2]]))
boxes.append(np.array([[0, 4, 0], [1, 5, 1]]))
boxes.append(np.array([[1.5, 4, 0], [2.5, 5, 1]]))
boxes.append(np.array([[5, 0, 2], [5.3, 5, 3]]))
boxes.append(np.array([[5, 1, 1], [5.3, 4, 2]]))
boxes.append(np.array([[5, 0, 0], [5.3, 4, 1]]))
boxes.append(np.array([[2, 2.5, 0], [5, 2.8, 3]]))

obstacles = list()
for box in boxes:
    obstacles.append(Obstacle(box[0, :], box[1, :]))

fig = plt.figure()
ax1 = fig.add_subplot(111, projection="3d")
ax1.set_xlim(0, 15)
ax1.set_ylim(-5, 10)
ax1.set_zlim(0, 15)
plt.rcParams['figure.figsize'] = 16, 16

# Plot the obstacles
for box in obstacles:
    plot_three_dee_box(box, ax=ax1)

real_trajectory['x'] = np.array(real_trajectory['x'])
real_trajectory['y'] = np.array(real_trajectory['y'])
real_trajectory['z'] = np.array(real_trajectory['z'])
point, = ax1.plot([real_trajectory['x'][0]], [real_trajectory['y'][0]], [real_trajectory['z'][0]], 'r+', markersize=20,
                  label='Quadrotor')
line, = ax1.plot([real_trajectory['x'][0]], [real_trajectory['y'][0]], [real_trajectory['z'][0]], 'g',
                 label='Real_Trajectory')

ax1.set_xlabel('x')
ax1.set_ylabel('y')
ax1.set_zlabel('z')
ax1.set_title('3D animate')
ax1.view_init(30, 35)
ax1.legend(loc='lower right')


def animate(i):
    line.set_xdata(real_trajectory['x'][:i + 1])
    line.set_ydata(real_trajectory['y'][:i + 1])
    line.set_3d_properties(real_trajectory['z'][:i + 1])
    point.set_xdata(real_trajectory['x'][i])
    point.set_ydata(real_trajectory['y'][i])
    point.set_3d_properties(real_trajectory['z'][i])


ani = animation.FuncAnimation(fig=fig, func=animate, frames=len(real_trajectory['x']), interval=1, repeat=False,
                              blit=False)
plt.show()
