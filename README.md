# Quadrotor 3D Planning and control
Repository for the project on quadrotor 3D planning and control using an RRT* method as global path planner, minimum snap optimization for the conversion of the global path into a trajectory tracked by a nonlinear Model Predictive Controller for local obstacle avoidance.

The present document describes the contents of the folders and the instructions on the usage of the different simulator files and functions

## How to download and test

Please, in order not to encounter any trouble while running the files, clone the repository to a folder and use pyCharm.
Notice that using a different IDE may require modifications to the import statements. Additionally, FORCESPRO needs to be
installed in the device and the current libraries with respective versions need to be installed (in a virtual environment
is enough):

- CADADI: 3.5.5
- NUMPY: 1.19.5 (versions 1.20 and above not supported due to CADADI incompatibilities)
- MATPLOTLIB: 3.3.4
- SCIPY: 1.6.3
- FORCESPRO: 1.5.0

## Scenarios

The current algorithm has been developed and tested in four different scenarios, represented in the following pictures. 
Please, note that the obstacle avoidance with an additional obstacle (not considered in global planning) has only been
tested on scenario 2, which was observed to be sufficiently wide and ample so as to ensure complete collision avoidance.

| Scenario 0  | Scenario 1 |
|---|---|
<img src="./full_results_and_more/README%20photos/Scenario_0.png" alt="005" style="zoom: 50%;" /> | <img src="./full_results_and_more/README%20photos/Scenario_1.png" alt="004" style="zoom: 50%;" /> |

| Scenario 2 | Scenario 3 |
|---|---|
<img src="./full_results_and_more/README%20photos/Scenario_2.png" alt="005" style="zoom: 50%;" /> | <img src="./full_results_and_more/README%20photos/Scenario_3.png" alt="004" style="zoom: 50%;" /> |

## Simulator file usage

The file **simulator.py** depicts the trajectory tracking with the Geometric Nonlinear Controller. Multiple settings are
provided both for the RRT* and the minimum snap optimization, with their corresponding descriptions in the file itself.

The file **simulator_MPC.py** allows for the simulation of the MPC in multiple operation modes, but with more limited
options in terms of minimum snap. Please notice once again that, in order to perform collision avoidance with
an unpredicted obstacle (that is, an obstacle in the way of the trajectory), scenario 2 is the only one for
which the avoidance has been tested to be performed robustly.

## Further results

The folder full_results_and_more contains data related to some of the tests carried out during the evaluation of the 
solution as well as file summaries of the inspected results. The folder back_end contains data related to the trajectory 
tracking and obstacle avoidance; while the folder front_end contains the data and results of RRT and minimum snap.

### Results of RRT
| Scenario 0  | Scenario 1 |
|---|---|
<img src="./full_results_and_more/README%20photos/Scenario_0_4000_1_mod.png" alt="005" style="zoom: 50%;" /> | <img src="./full_results_and_more/README%20photos/Scenario_1_4000_1_mod.png" alt="004" style="zoom: 50%;" /> |

### Results of minimum snap
| Scenario 0  | Scenario 1 |
|---|---|
<img src="./full_results_and_more/README%20photos/min_snap_scenario_0_penalty_10000.jpg" alt="005" style="zoom: 50%;" /> | <img src="./full_results_and_more/README%20photos/min_snap_scenario_1_penalty_2500_mod.jpg" alt="004" style="zoom: 50%;" /> |

### Results of MPC
| Path following  | Local avoidance |
|---|---|
<img src="./full_results_and_more/README%20photos/mpc_following_scenario4.gif" alt="005" style="zoom: 50%;" /> | <img src="./full_results_and_more/README%20photos/mpc_local_avoidance_scenario4.gif" alt="004" style="zoom: 50%;" /> |

## License 

Distributed under the MIT License. See `LICENSE` for more infomation.

## Contact 

Rubén Martín Rodríguez - r.martinrodriguez@student.tudelft.nl  
Xinjie Liu - x.liu-47@student.tudelft.nl  
Yuezhe Zhang -y.zhang-130@student.tudelft.nl  
Paul Féry - p.h.fery-1@student.tudelft.nl

## References
- [Dynamic obstacle avoidance](https://ieeexplore.ieee.org/document/9274865)
- [Quaternions and dynamics](https://archive.org/details/arxiv-0811.2889/page/n5/mode/2up)
- [Cubic splines](https://pythonnumericalmethods.berkeley.edu/notebooks/chapter17.03-Cubic-Spline-Interpolation.html)
- [Matplotlib library for 3D representation](https://matplotlib.org/stable/api/_as_gen/mpl_toolkits.mplot3d.axes3d.Axes3D.html)
- [MPC cost matrix reference](https://github.com/b4sgren/mpc)
- [Kinodynamic motion planning for quadrotor-like aerial robots:](https://oatao.univ-toulouse.fr/20169/1/Boeuf.pdf)
- [MPC ETH](https://github.com/uzh-rpg/high_mpc)
- [Min snap aggressive indoors](https://dspace.mit.edu/bitstream/handle/1721.1/106840/Roy_Polynomial%20trajectory.pdf?sequence=1&isAllowed=y)
- [Min snap actuator constraint](https://www.researchgate.net/publication/259741166_Actuator_Constrained_Trajectory_Generation_and_Control_for_Variable-Pitch_Quadrotors)
- [More actuation constraints](https://escholarship.mcgill.ca/downloads/f1881r83x?locale=en)
- [RRT* for global planning and Path Oriented Prunging for local planning](https://ieeexplore.ieee.org/document/9019196)
- RRT-star article: [source](https://dspace.mit.edu/handle/1721.1/81442)
- Minimum snap article: [source](https://ieeexplore-ieee-org.tudelft.idm.oclc.org/stamp/stamp.jsp?tp=&arnumber=5980409&tag=1)
- MPC: [source](https://www-sciencedirect-com.tudelft.idm.oclc.org/science/article/pii/S0005109899002149?casa_token=EtRfAwnkYDUAAAAA:EAadMGgXlCD6tl9-J3qMGj7QPTF5t_8XDcqPwkkQ92rMBwqAzOZmewztJbQDFOSRI6yG7kmAhQ)
- [A RRT* based kinodynamic trajectory planning algorithm for Multirotor Micro Air Vehicle](https://ieeexplore.ieee.org/document/9277168)
- [Path Planning Followed by Kinodynamic Smoothing for Multirotor Aerial Vehicles](https://ieeexplore.ieee.org/document/9290162)
- [Model Predictive control-based trajectory planning for quadrotors with state and input constraints](https://ieeexplore-ieee-org.tudelft.idm.oclc.org/document/7832517)
- [Minimum Snap Trajectory Tracking for a Quadrotor UAV using Nonlinear Model Predictive Control](https://www.researchgate.net/publication/346782883_Minimum_Snap_Trajectory_Tracking_for_a_Quadrotor_UAV_using_Nonlinear_Model_Predictive_Control)
- [Geometric path following control for multirotor vehicles using nonlinear model predictive control and 3D spline paths](https://ieeexplore-ieee-org.tudelft.idm.oclc.org/document/7502541)
