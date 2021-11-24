# PDM_Project
Repository for the Planning and Decision Making Project Code

This README file will contain the links to the resources used in the project, references, links to tools used for developing the project or anything project-related in general that we want to gather in some place. Some general sections will be initially created, but more can be added, as well as subsections, when required.

## Project report

[View the report on Overleaf](https://www.overleaf.com/read/djjcwndjbvxw)

## Plotting resources

- [Matplotlib library for 3D representation](https://matplotlib.org/stable/api/_as_gen/mpl_toolkits.mplot3d.axes3d.Axes3D.html)

## Reference articles

### For dynamic environments

- A UAV Dynamic Path Planning Algorithm: [source](https://ieeexplore.ieee.org/document/9337581)
	- Q-Learining algorithm (similar to RL) for global planning + artificial potential fields for local (dynamic) planning
- Decision planning of a quadrotor in dynamical environment: [source](https://ieeexplore.ieee.org/document/8028421)
	- Strictly Q-Learning algorithm (similar to RL). In this case integrating the dynamic planning under obstacle velocity assumptions.
- Dynamic obstacle avoidance path planning for UAV: [source](https://ieeexplore.ieee.org/document/9274865)
	- RRT* (Also, nice comparison between RRT, RRT* and RRT* Informed) for global planning plus dynamic winidow for local planning.
- Real-Time Motion Planning for Quadrotor MAVs in Three-dimensional Complex Environment: [source](https://ieeexplore.ieee.org/document/9019196)
	- RRT* for global planning and Path Oriented Prunging for local planning.
- External Forces Resilient Safe Motion Planning for Quadrotor: [source](https://arxiv.org/pdf/2103.11178.pdf)
    - Application of MPC to counteract wind force disturbances.
- Dynamics analysis and time-optimal motion planning for unmanned quadrotor transportation systems: [source](https://pdf.sciencedirectassets.com/271456/1-s2.0-S0957415818X00026/1-s2.0-S0957415818300096/main.pdf?X-Amz-Security-Token=IQoJb3JpZ2luX2VjEGUaCXVzLWVhc3QtMSJHMEUCIFRv%2FcGPfbNfMmCTE9G64iIOJrdXL9H0ZGu51C4IMKTRAiEA8NxAkzNF3zZzyNc8MdTXUW0XuloWnAmv1Ovud5PMV8Uq%2BgMILhAEGgwwNTkwMDM1NDY4NjUiDEoPuDjH39DaqbvgrirXA1v75UDciCHFhNCi1CyecTnvdhQkC%2BljR%2Fl5e3oCTHvcR84L02JRDONYxGdQwHIJxZMHS00BGjVefxZziZgqHcdCTJanCSKEvAEkqgXvk8OoJTulZs2O9%2FIB18MAKz25vFbHWEI8vbXG5ByEZz1Fo4aYVuGNij8wRdcWClQpMYddQlEkbzBa3OwGDRhmgIZfsR41XYhGgmK38wgwFafPoI95zFM03glJ9XPS%2B0VheoPTWeAyt8zXfKl5NnXVHw2FOuHUN5WVAlJkbw84%2BS8oo9jINHH%2Bu8aCusGkQrkHhVCXoWHF6rQOCpvQoMy7EWIJTvjzM4nqohFatQFGpSa8wkLm7t241l94wFGnF57crEuCEwDakIO6kjWEH9LqhBJv0lZKsvMYFrSJnc1432OUlJAY2RTHUuplTDYUB%2B%2BdYn1nBMnLzd9V7c8WRoUXa%2FFUk6CpBR98x4xDVYd4%2F5rE5hxtg2Hb8uEUdA%2F%2BrY3YYs%2BGHMrP7R9bdNIDxwn6BtA8TGl04OrEQPrYB15zCgRXn9aetyVkU5zjfz68vrMVK89PL5G05yOj%2BOu9bvaRuWe4i5DOpVgUlvZJUGgcc25jk0mhi5TBu33QsImYutvsCKzwsRBUHM2dwzC7oO6MBjqlAdrKhDXksXJ1jECfEvX0ttkKMsVJS1uXa3fVtiSOjUjuegRCy9FWjhcy0vgciZKMubwNLSvKhvtIdoMOOSNyH%2BM1ZPIrYA91R5ndg%2F63fKyh8pXuBsdmO6U2KxDnnvr%2BM7FS1wFG6Zrdm8umDXGe%2FOp4PWkTVrUD%2BSzXC7nbZfISSio%2Ftnkf1QE9WVeaptdlNMZ9aDq5jVmvG5ghJobQTh7af6BRfw%3D%3D&X-Amz-Algorithm=AWS4-HMAC-SHA256&X-Amz-Date=20211122T133904Z&X-Amz-SignedHeaders=host&X-Amz-Expires=300&X-Amz-Credential=ASIAQ3PHCVTY7PTZOIPU%2F20211122%2Fus-east-1%2Fs3%2Faws4_request&X-Amz-Signature=401685a1ae12bcc196076f3551383c328821c40d97e61fd549dd229d8c4d5d2c&hash=29edc4ad39b3534e7c723753d88fb7a7c651cc631ba9dfc3267f429039b0f462&host=68042c943591013ac2b2430a89b270f6af2c76d8dfd086a07176afe7c76c2c61&pii=S0957415818300096&tid=spdf-9059ebfe-a743-403a-9278-7497ffc3b2b9&sid=2f8ce8b278f7c544729b9a84aa24447538bfgxrqb&type=client)
    - No MPC, no RRT, some entirely different approach it seems (not certain about its utility for our project, might be too advanced).

### For static environments

- A RRT* based kinodynamic trajectory planning algorithm for Multirotor Micro Air Vehicle: [source](https://ieeexplore.ieee.org/document/9277168)
    - RRT* with multiple considerations on the kinodynamics of the system
- Path Planning Followed by Kinodynamic Smoothing for Multirotor Aerial Vehicles (MAVs): [source](https://ieeexplore.ieee.org/document/9290162)
    - RRT*

## Useful projects for inspiration or reference

- [3D Motion Planning repo by abhiojha8](https://github.com/abhiojha8/3D-Motion-Planning)
- Kinodynamic motion planning for quadrotor-like aerial robots: [source](https://oatao.univ-toulouse.fr/20169/1/Boeuf.pdf)
    - A Thesis on quadrotors, covers a very wide range of information on the topic. Chapter on steering function included.
- Very extensive project on path planning algorithms, it does not use any library for collisions, but implements it itself. Can be useful for inspiration: [source](https://github.com/zhm-real/PathPlanning)

## Various

- Link to the first reference for the quaternion derivation of the quadrotor model: [Quaternions and dynamics](https://archive.org/details/arxiv-0811.2889/page/n5/mode/2up)
- Very interesting approach for obstacle definition and collision detection. [source](https://gdbooks.gitbooks.io/3dcollisions/content/Chapter1/)
- Clear and intuitive explanation of configuration space and DOF(degree of freedom). See chapter 2, Modern Robotics. [source](http://hades.mech.northwestern.edu/images/7/7f/MR.pdf)
- Explanation of potential functions and fields. See chapter 4, Principles of Robot Motion. [source](http://mathdep.ifmo.ru/wp-content/uploads/2018/10/Intelligent-Robotics-and-Autonomous-Agents-series-Choset-H.-et-al.-Principles-of-Robot-Motion_-Theory-Algorithms-and-Implementations-MIT-2005.pdf)

