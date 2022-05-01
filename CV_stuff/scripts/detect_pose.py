import cv2
import numpy as np
import matplotlib.pyplot as plt
from grip import GripPipeline
from casadi import Opti, sin, cos, tan, vertcat, mtimes, sumsqr, sum1, MX, dot
from mpl_toolkits import mplot3d

# define a video capture object
vid = cv2.VideoCapture(0)
processor = GripPipeline()

size_to_dist = lambda a: 0.01714102+15.74211494/a
# x = np.array([175000, 100000, 46550, 10500, 4500, 2300, 1400])
# y = np.array([0.07, 0.1, 0.15, 0.3, 0.45, 0.6, 0.75])
# x = 1/np.sqrt(x/np.pi)
# x = np.vstack((np.ones(len(x)), x))
# print(x.T)

K = np.array([[983.24210558   0.         549.07355146]
 [  0.         984.67352261 368.2628812 ]
 [  0.           0.           1.        ]])
K_inv = np.linalg.inv(K)
P = np.eye(3, 4)

# print(np.linalg.lstsq(x.T, y))
# plt.plot(x, y)
# plt.show()
failed_coords = []
movement = []
dists = []
prev_pose = np.arange(5)[1:]
while(True):
      
    # Capture the video frame
    # by frame
    ret, frame = vid.read()
  
    # Display the resulting frame
    
    processor.process(frame)
    origin = np.array([frame.shape[0]//2, frame.shape[1]//2])

    keypoints = [max(processor.find_blobs_output, key =lambda a: a.size)] if len(processor.find_blobs_output) else []
    im_with_keypoints = cv2.drawKeypoints(frame, keypoints, np.array([]), (0,255,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
    # the 'q' button is set as thes

    if len(keypoints):
        kp = keypoints[0]
        ball_xy = np.array([kp.pt[0], kp.pt[1]])

        
        dist = size_to_dist(kp.size/2)
        dists += [dist]
        

        cam_coord = K_inv@np.array([[ball_xy[0]],
                              [ball_xy[1]],
                              [1]])
        print(cam_coord)


        opti = Opti()
        world_coord = opti.variable(4)
        init_world_coord = prev_pose
        obj = mtimes((cam_coord - mtimes(P, world_coord)).T, (cam_coord - mtimes(P, world_coord)))
        opti.minimize(obj)
        opti.subject_to([pow(world_coord[0], 2) + pow(world_coord[1], 2) + pow(world_coord[2], 2) == dist*dist,
                         world_coord[3] == 1,
                         world_coord[2] >= 0])
        opti.set_initial(world_coord, init_world_coord)
        ###### CONSTRUCT SOLVER AND SOLVE ######
        opti.solver('ipopt')
        p_opts = {"expand": False, "print_time": False, "ipopt": {"print_level": 0}}
        s_opts = {"max_iter": 1e3}
        opti.solver('ipopt', p_opts, s_opts)
        try:
            sol = opti.solve()
            world_coord_val = sol.value(world_coord)
            movement += [world_coord_val]
            prev_pose = world_coord_val
            # print(world_coord_val)
        except:
            failed_coords += [cam_coord]
            print(cam_coord)
        # print(world_coord_val/world_coord_val[3])

    cv2.imshow('frame', im_with_keypoints)
    
    # quitting button you may use any
    # desired button of your choice
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
  
# After the loop release the cap object
vid.release()
# Destroy all the windows
cv2.destroyAllWindows()
print(failed_coords)

fig = plt.figure()
ax = plt.axes(projection='3d')
full_data = np.array(movement)
print(full_data)
# print(dists)
ax.plot3D(full_data[5:,0], full_data[5:,1], full_data[5:,2], 'gray')
ax.set_xlabel('x')
ax.set_ylabel('y')
ax.set_zlabel('z')
plt.show()