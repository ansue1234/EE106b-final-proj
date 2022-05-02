import numpy as np
import matplotlib.pyplot as plt
from matplotlib import cm
from get_goal import get_goal_pt
from matplotlib import cm

def draw_trajectory(trajectory):
    """
    trajectory: nx3 ndarray of location in space
    """
    fig = plt.figure()
    ax = plt.axes(projection='3d')
    ax.plot3D(trajectory[5:,0], trajectory[5:,1], trajectory[5:,2], 'blue')
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')
    plt.show()

def draw_voxels(voxels, voxel_values, obstacle, target, origin, threshold=1.0):
    """
    voxels: nxnxn representation of space with confidence score in each voxel
    """
    x, y, z = voxels

    viridis = cm.get_cmap('viridis', 12)

    # combine the objects into a single boolean array
    voxelarray = voxel_values > threshold

    # voxelarray = np.ones(np.array([voxel_values.shape[0], voxel_values.shape[1], voxel_values.shape[2]]), dtype=bool)
    colors = np.zeros(np.array([voxel_values.shape[0], voxel_values.shape[1], voxel_values.shape[2], 4]))
    for i in range(voxel_values.shape[0]):
        for j in range(voxel_values.shape[1]):
            for k in range(voxel_values.shape[2]):
                colors[i][j][k] = np.array(viridis(voxel_values[i][j][k]))
                colors[i][j][k][3] = min(3*voxel_values[i][j][k], 1)

    # and plot everything
    fig = plt.figure()
    ax = fig.add_subplot(projection='3d')
    ax.voxels(x, y, z, voxelarray[:-1, :-1, :-1], facecolors=colors[:-1, :-1, :-1, :], shade=False)
    cbar = fig.colorbar(cm.ScalarMappable(cmap=viridis), shrink=0.7, location="left")
    cbar.minorticks_on()
    ax.plot3D([origin[0], target[0]], [origin[1], target[1]], [origin[2], target[2]], color='g')
    ax.scatter([origin[0]], [origin[1]], [origin[2]], s= 40, color='b', label='Drone Position')
    ax.scatter([obstacle[0]], [obstacle[1]], [obstacle[2]], s= 40, color='y', label='Obstacle Position')
    ax.scatter([target[0]], [target[1]], [target[2]], s= 40, color='r', label='Escape Position')

    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')
    ax.set_title("Trajectory Probabilistic Region (P(collide) > {:.2f})".format(threshold))
    plt.legend()
    plt.show()


escape_point, voxels, voxel_vals = get_goal_pt(np.array([0, 0, 0]), np.array([1, 0, 0]), np.array([2.5, 0, 0]), t=1)
draw_voxels(voxels, voxel_vals, np.array([0, 0, 0]), escape_point, np.array([2.5, 0, 0]), threshold=0.0)
draw_voxels(voxels, voxel_vals, np.array([0, 0, 0]), escape_point, np.array([2.5, 0, 0]), threshold=0.2)
draw_voxels(voxels, voxel_vals, np.array([0, 0, 0]), escape_point, np.array([2.5, 0, 0]), threshold=0.4)

escape_point, voxels, voxel_vals = get_goal_pt(np.array([0, 0, 0]), np.array([5, 0, 0]), np.array([2.5, 0, 0]), t=1)
draw_voxels(voxels, voxel_vals, np.array([0, 0, 0]), escape_point, np.array([2.5, 0, 0]), threshold=0.0)
draw_voxels(voxels, voxel_vals, np.array([0, 0, 0]), escape_point, np.array([2.5, 0, 0]), threshold=0.2)
draw_voxels(voxels, voxel_vals, np.array([0, 0, 0]), escape_point, np.array([2.5, 0, 0]), threshold=0.4)

escape_point, voxels, voxel_vals = get_goal_pt(np.array([0, 0, 0]), np.array([1, 1, 1]), np.array([2.5, 0, 0]), t=1)
draw_voxels(voxels, voxel_vals, np.array([0, 0, 0]), escape_point, np.array([2.5, 0, 0]), threshold=0.0)
draw_voxels(voxels, voxel_vals, np.array([0, 0, 0]), escape_point, np.array([2.5, 0, 0]), threshold=0.2)
draw_voxels(voxels, voxel_vals, np.array([0, 0, 0]), escape_point, np.array([2.5, 0, 0]), threshold=0.4)