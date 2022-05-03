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
    confidence_x = norm.sf(x=np.abs(a_x_desired), loc = 0, scale = 2)
    confidence_y = norm.sf(x=np.abs(a_y_desired), loc = 0, scale = 2)
    confidence_z = (norm.sf(x=np.abs(a_z_desired), loc = 0, scale = 2) + norm.sf(x=np.abs(a_z_desired), loc = 9.81, scale = 2))/2.0

    return (confidence_x + confidence_y + confidence_z)/3.0

def get_goal_pt(object_pose, object_velocity, drone_pose, resolution=11, look_ahead_dist=2.5, t=1, threshold=0.2, visualize=True):
    dist_increment = look_ahead_dist*2/resolution
    origin = resolution//2
    drone_x, drone_y, drone_z = drone_pose[0], drone_pose[1], drone_pose[2]
    closest_voxel = np.array([np.inf, np.inf, np.inf])
    if visualize:
        voxel_grid = np.zeros((3, resolution, resolution, resolution))
        confidence_grid = np.zeros((resolution, resolution, resolution))
        for i in range(resolution):
            for j in range(resolution):
                for k in range(resolution):
                    delta_x = (i - origin)*dist_increment
                    delta_y = (j - origin)*dist_increment
                    delta_z = (origin - k)*dist_increment
                    voxel_pose = np.array([drone_x + delta_x, drone_y + delta_y, drone_z + delta_z])
                    voxel_confidence = confidence(object_pose, object_velocity, voxel_pose, t=t)
                    voxel_pt = np.array([delta_x, delta_y, delta_z])
                    voxel_grid[0][i][j][k] = (drone_pose + voxel_pt)[0]
                    voxel_grid[1][i][j][k] = (drone_pose + voxel_pt)[1]
                    voxel_grid[2][i][j][k] = (drone_pose + voxel_pt)[2]
                    confidence_grid[i][j][k] = voxel_confidence
                    if voxel_confidence < threshold:
                        if np.linalg.norm(voxel_pt) < np.linalg.norm(closest_voxel):
                            closest_voxel = voxel_pt
                            # confidence_grid[i][j][k][3] = 1
        return drone_pose + closest_voxel, voxel_grid, confidence_grid
    else:
        voxel_grid = np.zeros((3, resolution, resolution, resolution))
        confidence_grid = np.zeros((resolution, resolution, resolution))
        queue = [(origin, origin, origin)]
        closest_voxel = None
        while len(queue):
            i, j, k = queue[0]
            queue = queue[1:]
            if (i >= resolution or i < 0) or  (j >= resolution or j < 0) or (k >= resolution or k < 0):
                continue
            if confidence_grid[i][j][k] != -1:
                continue
            delta_x = (i - origin)*dist_increment
            delta_y = (j - origin)*dist_increment
            delta_z = (origin - k)*dist_increment 
            voxel_pose = np.array([drone_x + delta_x, drone_y + delta_y, drone_z + delta_z])
            voxel_confidence = confidence(object_pose, object_velocity, voxel_pose, t=t)
            voxel_pt = np.array([delta_x, delta_y, delta_z])
            voxel_grid[0][i][j][k] = voxel_pose[0]
            voxel_grid[1][i][j][k] = voxel_pose[1]
            voxel_grid[2][i][j][k] = voxel_pose[2]
            confidence_grid[i][j][k] = voxel_confidence
            if not closest_voxel and voxel_confidence <= threshold:
                closest_voxel = voxel_pose
            queue += [(i + 1, j + 1, k), (i + 1, j, k), (i + 1, j - 1, k),
                      (i, j + 1, k), (i, j - 1, k),
                      (i - 1, j + 1, k), (i - 1, j, k), (i - 1, j - 1, k),
                      (i + 1, j + 1, k + 1), (i + 1, j, k + 1), (i + 1, j - 1, k + 1),
                      (i, j + 1, k + 1), (i, j, k + 1), (i, j - 1, k + 1),
                      (i - 1, j + 1, k + 1), (i - 1, j, k + 1), (i - 1, j - 1, k + 1),
                      (i + 1, j + 1, k - 1), (i + 1, j, k - 1), (i + 1, j - 1, k - 1),
                      (i, j + 1, k - 1), (i, j, k - 1), (i, j - 1, k - 1),
                      (i - 1, j + 1, k - 1), (i - 1, j, k - 1), (i - 1, j - 1, k - 1),
                      ]
            

        # for i in range(resolution):
        #     for j in range(resolution):
        #         for k in range(resolution):
        #             delta_x = (i - origin)*dist_increment
        #             delta_y = (j - origin)*dist_increment
        #             delta_z = (origin - k)*dist_increment
        #             voxel_pose = np.array([drone_x + delta_x, drone_y + delta_y, drone_z + delta_z])
        #             voxel_confidence = confidence(object_pose, object_velocity, voxel_pose, t=t)
        #             voxel_pt = np.array([delta_x, delta_y, delta_z])
        #             voxel_grid[0][i][j][k] = (drone_pose + voxel_pt)[0]
        #             voxel_grid[1][i][j][k] = (drone_pose + voxel_pt)[1]
        #             voxel_grid[2][i][j][k] = (drone_pose + voxel_pt)[2]
        #             confidence_grid[i][j][k] = voxel_confidence
        #             if voxel_confidence < threshold:
        #                 if np.linalg.norm(voxel_pt) < np.linalg.norm(closest_voxel):
        #                     closest_voxel = voxel_pt
        #                     # confidence_grid[i][j][k][3] = 1
        return closest_voxel, voxel_grid, confidence_grid