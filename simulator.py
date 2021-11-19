import os
import sys
##################################################################
# deal with the folders
url1=os.path.join(os.getcwd(),"model/")
url2=os.path.join(os.getcwd(),"traj_handles_ro47001/")
url3=os.path.join(os.getcwd(),"map/")
sys.path.append(url1)
sys.path.append(url2)
sys.path.append(url3)
#################################################################
import numpy as np
from matplotlib import pyplot as plt
import mpl_toolkits.mplot3d.axes3d as p3
from matplotlib import animation
from quadrotor import Quadrotor
from nonlinear_controller import GeometricControlller
from tj_handle_tud import tj_tud, get_T_tud
from tj_handle_circle import tj_circle, get_T_circle
from tj_handle_diamond import tj_diamond, get_T_diamond
from tj_handle_hover import tj_hover, get_T_hover
from map_3d import create_voxmap


#################################################################
'''
You can choose trajectory here!
'''
#----------------------------------------------------------------

tj_handle = tj_tud
T = get_T_tud()

#----------------------------------------------------------------

#tj_handle = tj_circle
#T = get_T_circle()

#----------------------------------------------------------------

#tj_handle = tj_diamond
#T = get_T_diamond()

#----------------------------------------------------------------

#tj_handle = tj_hover
#T = get_T_hover()

#################################################################


env = Quadrotor()
# circle trajectory has different initial position
if tj_handle == tj_circle:
    current_state = env.reset(position=[5, 0, 0])
else:
    current_state = env.reset(position=[0, 0, 0])
dt = 0.01
t = 0
policy = GeometricControlller()
time_step = 1e-2
iterations = int(T/time_step)
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
    total_SE += (np.sum((obs['x'] - state_des['pos'])**2)*time_step)
    total_energy += (np.sum(cmd_rotor_speeds**2)*time_step)
    
############################################################################
print("Sum of tracking error (integration): ", total_SE)
print("Total time: ", t)
print("Sum of energy consumption (integration)", total_energy)
############################################################################

############################################################################
# plot nice maps!

fig = plt.figure()
#ax1 = fig.gca(projection='3d')
ax1 = p3.Axes3D(fig) # 3D place for drawing
plt.rcParams['figure.figsize'] = 16, 16

############################################################################
# plot of the map
filename = 'colliders.csv'
data = np.loadtxt(filename, delimiter=',', dtype=np.float64, skiprows=2)
print(data)
voxmap = create_voxmap(data, 15)
print(voxmap.shape)

ax1.voxels(voxmap, edgecolor='k')
ax1.set_xlim(voxmap.shape[0], 0)
ax1.set_ylim(0, voxmap.shape[1])
# add a bit to z-axis height for visualization
ax1.set_zlim(0, voxmap.shape[2]+20)

#plt.xlabel('North')
#plt.ylabel('East')
############################################################################

real_trajectory['x'] = np.array(real_trajectory['x'])
real_trajectory['y'] = np.array(real_trajectory['y'])
real_trajectory['z'] = np.array(real_trajectory['z'])
point, = ax1.plot([real_trajectory['x'][0]], [real_trajectory['y'][0]], [real_trajectory['z'][0]], 'r+', markersize=20, label='Quadrotor')
line, = ax1.plot([real_trajectory['x'][0]], [real_trajectory['y'][0]], [real_trajectory['z'][0]], 'g', label='Real_Trajectory')

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

ani = animation.FuncAnimation(fig=fig,
                              func=animate,
                              frames=len(real_trajectory['x']),
                              interval=1,
                              repeat=False,
                              blit=False)
plt.show()