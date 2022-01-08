# PDM_Project
Repository for the Planning and Decision Making Project Code

This README file will contain the links to the resources used in the project, references, links to tools used for developing the project or anything project-related in general that we want to gather in some place. Some general sections will be initially created, but more can be added, as well as subsections, when required.

## Project report

[View the report on Overleaf](https://www.overleaf.com/read/djjcwndjbvxw)

## Plotting resources

- [Matplotlib library for 3D representation](https://matplotlib.org/stable/api/_as_gen/mpl_toolkits.mplot3d.axes3d.Axes3D.html)

## How to

### RRT*

- The RRT_star class can be instantiated using as compulsory arguments the initial position, introduced as a numpy array,the number of iterations of the algorithm, the list of obstacle objects in the environment.
- The optional arguments are:
  - The threshold for self-connection with the goal, which is defined with a default value of 0.5
  - The axis for animation, if none are provided, no animation of the RRT will be carried out and only the final path will be represented at the end of the simulation by "plot_all"
  - A margin to be left with respect to the obstacles (mostly required for the MPC).
- In order to execute the algorithm, use the **find_path** function, which takes as arguments the goal point in the same format as the start point and the map_boundaries to which the samples will be constrained. If the RRT_star object has been instantiated with an axis for animation, the animation will start immediately as this function is being called.
- Once the algorithm has run, the path can be obtained in two different ways:
  - **get_path**: will return the complete path found by the RRT* algorithm
  - **get_straight_path**: modification of the complete path found by the algorithm in which, starting from the goal, the points are consecutively connected with the furthest point in the path without colliding with obstacles. ***It is highly recommendable to use it with advanced versions of the minimum snap (actuator constraints) to reduce the computation time***.
- A static plot of the generated tree can be obtained, by means of the "plotTree" method. If a solution has been found, the corresponding path is highlighted.

### Minimum snap

- The function to call the minimum snap optimization is **min_snap_optimizer_3d**.
- The only two parameters necessarily required are the path from the RRT* algorithm and the penalty (integer) that will be imposed to time in the cost function.
- The remaining parameters are optional and determine the capabilities and guarantees of the algorithm
  - **time_optimal**: when set to **True**, the time allocated for each segment of the path will be optimized. In such case, any of the other options can be used. When set to False, the time allocated for each segment will be calculated proportionally to the total length of the path and a given total time for completing the path, given as an additional argument.
  - **total_time**: total time required to complete the path, can only be used when **time_optimal** is set to **False**.
  - **act_const**: can only be set to **True** when **time_optimal** so is, since otherwise it could lead to conflicts with the total time requirement. It performs the optimization ensuring that the trajectory will not require the drone to exceed the limits of actuation. Please, note that this will slow down the optimization. When set to **False** a value of 2700 for the penalty is selected, which allows for a safe profile of the cost function and convergence of the solution without excessive penalty on time.
  - **check_collision**: can be used regardless of the value of **time_optimal**. It will check for collisions with the provided list of obstacles and modify the path to avoid collisions, if detected. Note that a single collision will twofold or more the computation time.
  - **obstacles**: only required if **check_collision** is set to **True**, must be a list of Obstacle objects to check collision with.

Therefore, the more constraints and complex the optimization, increased computation time, but better guarantees. If the completion time of the path is desired to be changed, this can be achieved by setting **time_optimal** to **False** and providing the required completion time. An alternative option is to reduce the **penalty** value, indistinctly of whether actuation constraints are used or not. However, for values of the penalty larger than 2700 no change will be done if **act_const** is set to **False** as the value will be capped.

## Reference articles

### Algorithm definition articles

- RRT-star article: [source](https://dspace.mit.edu/handle/1721.1/81442)
- Minimum snap article: [source](https://ieeexplore-ieee-org.tudelft.idm.oclc.org/stamp/stamp.jsp?tp=&arnumber=5980409&tag=1)
- For MPC: [source](https://www-sciencedirect-com.tudelft.idm.oclc.org/science/article/pii/S0005109899002149?casa_token=EtRfAwnkYDUAAAAA:EAadMGgXlCD6tl9-J3qMGj7QPTF5t_8XDcqPwkkQ92rMBwqAzOZmewztJbQDFOSRI6yG7kmAhQ)

### References inserted in the project report

- Dynamic obstacle avoidance path planning for UAV: [source](https://ieeexplore.ieee.org/document/9274865)
    - RRT* (Also, nice comparison between RRT, RRT* and RRT* Informed) for global planning plus dynamic winidow for local planning.
- A RRT* based kinodynamic trajectory planning algorithm for Multirotor Micro Air Vehicle: [source](https://ieeexplore.ieee.org/document/9277168)
    - RRT* with multiple considerations on the kinodynamics of the system
- Path Planning Followed by Kinodynamic Smoothing for Multirotor Aerial Vehicles (MAVs): [source](https://ieeexplore.ieee.org/document/9290162)
    - RRT*
- Model Predictive control-based trajectory planning for quadrotors with state and input constraints: [source](https://ieeexplore-ieee-org.tudelft.idm.oclc.org/document/7832517)
    - MPC with RRT
- Minimum Snap Trajectory Tracking for a Quadrotor UAV using Nonlinear Model Predictive Control [source](https://www.researchgate.net/publication/346782883_Minimum_Snap_Trajectory_Tracking_for_a_Quadrotor_UAV_using_Nonlinear_Model_Predictive_Control)
- Geometric path following control for multirotor vehicles using nonlinear model predictive control and 3D spline paths: [source](https://ieeexplore-ieee-org.tudelft.idm.oclc.org/document/7502541)

### For dynamic environments

- Real-Time Motion Planning for Quadrotor MAVs in Three-dimensional Complex Environment: [source](https://ieeexplore.ieee.org/document/9019196)
	- RRT* for global planning and Path Oriented Prunging for local planning.
- External Forces Resilient Safe Motion Planning for Quadrotor: [source](https://arxiv.org/pdf/2103.11178.pdf)
    - Application of MPC to counteract wind force disturbances.

### Minimum snap sources

- Polynomial Trajectory Planning for Aggressive Quadrotor Flight in Dense Indoor Environments: [source](https://dspace.mit.edu/bitstream/handle/1721.1/106840/Roy_Polynomial%20trajectory.pdf?sequence=1&isAllowed=y)
    - Polynomial trajectory
    - Time allocation optimization
    - we can also get some inspiration for our report from the article's structure
    - The c++ code of the nonlinear programming based time allocation and trajectory generation can be found at: [ethz-asl](https://github.com/ethz-asl/mav_trajectory_generation) 

- Actuator Constrained Trajectory Generation and Control for Variable-Pitch Quadrotors: [source](https://www.researchgate.net/publication/259741166_Actuator_Constrained_Trajectory_Generation_and_Control_for_Variable-Pitch_Quadrotors)

- Constrained Control Allocation Approaches in Trajectory Control of a Quadrotor Under Actuator Saturation: [source](https://escholarship.mcgill.ca/downloads/f1881r83x?locale=en)

### Useful projects for inspiration or reference

- [3D Motion Planning repo by abhiojha8](https://github.com/abhiojha8/3D-Motion-Planning)

- Kinodynamic motion planning for quadrotor-like aerial robots: [source](https://oatao.univ-toulouse.fr/20169/1/Boeuf.pdf)
    - A Thesis on quadrotors, covers a very wide range of information on the topic. Chapter on steering function included.

- Very extensive project on path planning algorithms, it does not use any library for collisions, but implements it itself. Can be useful for inspiration: [source](https://github.com/zhm-real/PathPlanning)

- Non-linear MPC control from University of Zuerich, they used casadi optimization solver and non-linear quadrotor model.We can refer to their code for setting optimizer and Q, R matrices. [source](https://github.com/uzh-rpg/high_mpc)

- Linear MPC, we can refer to their Q,R matrices.[source](https://github.com/b4sgren/mpc)

## Various

- Link to the first reference for the quaternion derivation of the quadrotor model: [Quaternions and dynamics](https://archive.org/details/arxiv-0811.2889/page/n5/mode/2up)

- Very interesting approach for obstacle definition and collision detection. [source](https://gdbooks.gitbooks.io/3dcollisions/content/Chapter1/)

- Explanation of cubic spline interpolation from Python Numerical Methods, Berkley. [source](https://pythonnumericalmethods.berkeley.edu/notebooks/chapter17.03-Cubic-Spline-Interpolation.html)
