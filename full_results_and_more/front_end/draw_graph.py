#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import numpy as np
import matplotlib.pyplot as plt

'''
# computation time for RRT
rrt_time_sce0_goal1 = np.array([52.82, 99.32, 261.9])
rrt_time_sce0_goal2 = np.array([43.09, 93.22, 262.72])
rrt_time_sce1_goal1 = np.array([79.42, 207.07, 437.8])
rrt_time_sce1_goal2 = np.array([91.0, 207.28, 468.86])
rrt_time_sce5_goal1 = np.array([68.97, 142.05, 325.05])
rrt_time_sce5_goal2 = np.array([68.33, 139.49, 346.26])

iterations = np.array([1500, 2500, 4000])

plt.figure(1)
plt.plot(iterations, rrt_time_sce0_goal1, 's-', color = 'r', label = 'scenario 0 goal 1')
plt.plot(iterations, rrt_time_sce0_goal2, 's-', color ='b', label = 'scenario 0 goal 2')
plt.plot(iterations, rrt_time_sce1_goal1, 'o-', color ='c', label = 'scenario 1 goal 1')
plt.plot(iterations, rrt_time_sce1_goal2, 'o-', color ='g', label = 'scenario 1 goal 2')
plt.plot(iterations, rrt_time_sce5_goal1, '*-', color ='m', label = 'scenario 5 goal 1')
plt.plot(iterations, rrt_time_sce5_goal2, '*-', color ='y', label = 'scenario 5 goal 2')
plt.xlim(1000, 4000)

plt.xlabel('Number of iterations')
plt.ylabel('Computation time (s)')
plt.title('Computation time of RRT with different iterations')
plt.legend(loc='best')
plt.show()


# length of path compared with simplified path
len_sce0_g1 = (24.78 + 25.087 + 24.53) / 3
simplify_len_sce0_g1 = (24.61 + 24.49 + 24.18) / 3
len_sce1_g1 = (10.047 + 10.528 + 10.603) / 3
simlify_len_sce1_g1 = (10.363 + 10.428 + 10.56) / 3
len_sce5_g1 = (15.45 + 14.66 + 14.90) / 3
simplify_len_sce5_g1 = (15.09 + 14.5 + 14.51) / 3

plt.figure(2)
barWidth = 0.25
sce = np.arange(3)
simplify_sce = [x + barWidth for x in sce]
length_path = np.array([len_sce0_g1, len_sce1_g1, len_sce5_g1])
simplify_length_path = np.array([simplify_len_sce0_g1, simlify_len_sce1_g1, simplify_len_sce5_g1])
plt.bar(sce, length_path, color ='r', width = barWidth, label = 'path')
plt.bar(simplify_sce, simplify_length_path, color ='b', width = barWidth, label = 'simplified path')

plt.xlabel("Differnet scenarios")
plt.ylabel("Length of path (m)")
plt.xticks([r + barWidth/2 for r in range(3)],
        ['sce 0', 'sce 1', 'sce 5'])
plt.title("Length of path compared with simplified path in different scenarios")
plt.legend()
plt.show()
'''

# computation time for minimum snap
minisnap_time_sce0 = np.array([24.28, 23.25, 998.85])
minisnap_time_sce1 = np.array([11.41, 45.96, 475.17])
minisnap_time_sce5 = np.array([10.46, 8.87, 274.5])
# trajectory time for minimum snap
traj_time_sce0 = np.array([7.85, 7.26, 6.228])
traj_time_sce1 = np.array([5.39, 5.18, 4.28])
traj_time_sce5 = np.array([5.78, 5.36, 4.59])

penalty = np.array([1250, 2500, 10000])
fig, ax1 = plt.subplots()
ax2 = ax1.twinx()

ax1.plot(penalty, minisnap_time_sce0, 's-', color = 'r', label = 'computation time in sce 0')
ax1.plot(penalty, minisnap_time_sce1, 'o-', color = 'r', label = 'computation time in sce 1')
ax1.plot(penalty, minisnap_time_sce5, '^-', color = 'r', label = 'computation time in sce 5')

ax1.set_xlabel('Penalty')
ax1.set_ylabel('Computation time (s)')
ax1.set_ylim(0,1300)
ax1.legend(loc='upper right')

ax2.plot(penalty, traj_time_sce0, 's-', color = 'b', label = 'trajectory time in sce 0')
ax2.plot(penalty, traj_time_sce1, 'o-', color = 'b', label = 'trajectory time in sce 1')
ax2.plot(penalty, traj_time_sce5, '^-', color = 'b', label = 'trajectory time in sce 5')

ax2.set_ylabel('Trajectory time (s)')
ax2.set_ylim(3,10)
ax2.legend(loc='upper left')
plt.title('Computation and trajectory time of minimum snap with different penalty')
plt.show()
