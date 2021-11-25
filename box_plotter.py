# IMPORTS #############################################################################################################

import numpy as np
import matplotlib.pyplot as plt
from Obstacle import Obstacle
from mpl_toolkits.mplot3d import Axes3D

# DEFINITIONS #########################################################################################################


def plot_three_dee_box(points, ax=None, rgb=(1, 0, 0), opacity=0.6, show=False):
    """
    This function takes 2 3D points, defining a 3D orthogonal box, and plots it.

    :param points: the box-defining points, a set of two 3D coordinates,
    format can be a np.array of shape (2, 3), or an Obstacle object.
    :param ax: the parent plotting environment, if none is provided, one will be created automatically
    :param rgb: the color to use, by default red, format is a tuple of RGB values
    :param opacity: determines the opacity og the box, format is a float, must be between 0 and 1
    :param show: if set to True, the function will automatically plot the box
    :return: None
    """
    if type(points) == Obstacle:
        points = np.array([points.point_min_, points.point_max_])

    if ax is None:
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')

    x = points[:, 0]    # the box's x-coordinate extremities
    y = points[:, 1]    # the box's y-coordinate extremities
    z = points[:, 2]    # the box's z-coordinate extremities

    yy_x, zz_x = np.meshgrid(y, z)      # the grids of points for the planes perpendicular to the x-axis
    xx_y, zz_y = np.meshgrid(x, z)      # the grids of points for the planes perpendicular to the y-axis
    xx_z, yy_z = np.meshgrid(x, y)      # the grids of points for the planes perpendicular to the z-axis

    # plotting all 6 faces of the cuboid
    ax.plot_surface(points[0, 0].reshape(1, 1), yy_x, zz_x, color=rgb, alpha=opacity)
    ax.plot_surface(points[1, 0].reshape(1, 1), yy_x, zz_x, color=rgb, alpha=opacity)
    ax.plot_surface(xx_y, points[0, 1].reshape(1, 1), zz_y, color=rgb, alpha=opacity)
    ax.plot_surface(xx_y, points[1, 1].reshape(1, 1), zz_y, color=rgb, alpha=opacity)
    ax.plot_surface(xx_z, yy_z, points[0, 2].reshape(1, 1), color=rgb, alpha=opacity)
    ax.plot_surface(xx_z, yy_z, points[1, 2].reshape(1, 1), color=rgb, alpha=opacity)

    if show:
        plt.show()


# MAIN FUNCTION #######################################################################################################


def main():
    """This function takes the boxes defining the layout of the factory, and plots them in 3D space."""
    # For visualization, a plane from [0, 0, 0] to [0, 5.3, 3] and a ground plane can be added.
    # For collision detection, those planes will be implicit in the search space.

    # large_wall_1 = np.array([[0, 5, 0], [14, 5.3, 3]])
    # large_wall_2 = np.array([[14, 5, 0], [15, 5.3, 2]])
    # table_1 = np.array([[0, 4, 0], [1, 5, 1]])
    # table_2 = np.array([[1.5, 4, 0], [2.5, 5, 1]])
    # entrance_1 = np.array([[5, 0, 2], [5.3, 5, 3]])
    # entrance_2 = np.array([[5, 1, 1], [5.3, 4, 2]])
    # entrance_3 = np.array([[5, 0, 0], [5.3, 4, 1]])
    # mid_wall = np.array([[2, 2.5, 0], [5, 2.8, 3]])

    large_wall_1 = Obstacle(np.array([0, 5, 0]), np.array([14, 5.3, 3]))
    large_wall_2 = Obstacle(np.array([14, 5, 0]), np.array([15, 5.3, 2]))
    table_1 = Obstacle(np.array([0, 4, 0]), np.array([1, 5, 1]))
    table_2 = Obstacle(np.array([1.5, 4, 0]), np.array([2.5, 5, 1]))
    entrance_1 = Obstacle(np.array([5, 0, 2]), np.array([5.3, 5, 3]))
    entrance_2 = Obstacle(np.array([5, 1, 1]), np.array([5.3, 4, 2]))
    entrance_3 = Obstacle(np.array([5, 0, 0]), np.array([5.3, 4, 1]))
    mid_wall = Obstacle(np.array([2, 2.5, 0]), np.array([5, 2.8, 3]))

    boxes = list()

    boxes.append(large_wall_1)
    boxes.append(large_wall_2)
    boxes.append(table_1)
    boxes.append(table_2)
    boxes.append(entrance_1)
    boxes.append(entrance_2)
    boxes.append(entrance_3)
    boxes.append(mid_wall)

    fig = plt.figure()
    ax = fig.add_subplot(111, projection="3d")
    ax.set_xlim(0, 15)
    ax.set_ylim(-5, 10)
    ax.set_zlim(0, 15)

    for box in boxes:
        plot_three_dee_box(box, ax=ax)
    plt.show()


if __name__ == '__main__':
    main()
