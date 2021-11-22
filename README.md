# PDM_Project
Repository for the Planning and Decision Making Project Code

This README file will contain the links to the resources used in the project, references, links to tools used for developing the project or anything project-related in general that we want to gather in some place. Some general sections will be initially created, but more can be added, as well as subsections, when required.

## Project report

Go to Overleaf (I found no link for this one as I think it will depend on the person)

## Plotting resources

- [Matplotlib library for 3D representation](https://matplotlib.org/stable/api/_as_gen/mpl_toolkits.mplot3d.axes3d.Axes3D.html)

## Reference articles

### For dynamic environmentsç

- A UAV Dynamic Path Planning Algorithm: [source](https://ieeexplore.ieee.org/document/9337581)
	- Q-Learining algorithm (similar to RL) for global planning + artificial potential fields for local (dynamic) planning
- Decision planning of a quadrotor in dynamical environment: [source](https://ieeexplore.ieee.org/document/8028421)
	- Strictly Q-Learning algorithm (similar to RL). In this case integrating the dynamic planning under obstacle velocity assumptions.
- Dynamic obstacle avoidance path planning for UAV: [souorce](https://ieeexplore.ieee.org/document/9274865)
	- RRT* (Also, nice comparison between RRT, RRT* and RRT* Informed) for global planning plus dynamic winidow for local planning.
- Real-Time Motion Planning for Quadrotor MAVs in Three-dimensional Complex Environment: [source](https://ieeexplore.ieee.org/document/9019196)
	- RRT* for global planning and Path Oriented Prunging for local planning.


### For static environments

- A RRT* based kinodynamic trajectory planning algorithm for Multirotor Micro Air Vehicle: [source](https://ieeexplore.ieee.org/document/9277168)
	- RRT* with multiple considerations on the kinodynamics of the system
- Path Planning Followed by Kinodynamic Smoothing for Multirotor Aerial Vehicles (MAVs): [source](https://ieeexplore.ieee.org/document/9290162)
	- RRT* 

## Useful projects for inspiration or reference

- [3D Motion Planning repo by abhiojha8](https://github.com/abhiojha8/3D-Motion-Planning)

## Various

- Link to the first reference for the quaternion derivation of the quadrotor model: [Quaternions and dynamics](https://archive.org/details/arxiv-0811.2889/page/n5/mode/2up)


