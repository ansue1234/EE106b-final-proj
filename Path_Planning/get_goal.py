import numpy as np
from scipy.stats import norm

def confidence(object_pose, object_velocity, target_pose, t=1):
    obj_x, obj_y, obj_z = object_pose[0], object_pose[1], object_pose[2]
    obj_vx, obj_vy, obj_vz = object_velocity[0], object_velocity[1], object_velocity[2]
    target_x, target_y, target_z = target_pose[0], target_pose[1], target_pose[2]

    # finding desired acceration to the drone
    find_a = lambda x_d, x_o, v, t: 2*(x_d - x_o - v*t)/t**2
    
    a_x_desired = find_a(target_x, obj_x, obj_vx, t)
    a_y_desired = find_a(target_y, obj_y, obj_vy, t)
    a_z_desired = find_a(target_z, obj_z, obj_vz, t)

    # calculating confidence via 1-cdf
    confidence_x = norm.sf(x=np.abs(a_x_desired), loc = 0, scale = 3)
    confidence_y = norm.sf(x=np.abs(a_y_desired), loc = 0, scale = 3)
    confidence_z = (norm.sf(x=np.abs(a_z_desired), loc = 0, scale = 3) + norm.sf(x=np.abs(a_z_desired), loc = 9.81, scale = 3))/2.0

    return (confidence_x + confidence_y + confidence_z)/3.0

def get_goal_pt(object_pose, object_velocity, drone_pose, resolution=11.0, look_ahead_dist=2.5, t=1, threshold=0.5):
    voxel_grid = np.zeros((resolution, resolution, resolution))
    dist_increment = look_ahead_dist*2/resolution
    origin = resolution//2
    drone_x, drone_y, drone_z = drone_pose[0], drone_pose[1], drone_pose[2]
    closest_voxel = None
    for i in range(resolution):
        for j in range(resolution):
            for k in range(resolution):
                delta_x = (i - origin)*dist_increment
                delta_y = (j - origin)*dist_increment
                delta_z = (origin - k)*dist_increment

                voxel_pose = np.array([drone_x + delta_x, drone_y + delta_y, drone_z + delta_z])
                voxel_grid[i][j][k] = confidence(object_pose, object_velocity, voxel_pose)








