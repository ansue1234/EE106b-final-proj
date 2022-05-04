import imp
import rospy
# from std_msgs.msg import String


# for trajectory
import numpy as np
import matplotlib.pyplot as plt
from casadi import Opti, sin, cos, tan, vertcat, mtimes, sumsqr, sum1, MX, dot
from mpl_toolkits import mplot3d
from std_msgs.msg import Int8 
from darknet_ros_msgs.msg import BoundingBoxes
from sensor_msgs.msg import Image
from geometry_msgs import Twist
from geometry_msgs import Pose
from Object_Avoidance import ObjectStateMsg
import time

# information for trajectory plotting
K = np.array([[983.24210558  , 0.     ,    549.07355146],
[  0.      ,   984.67352261, 368.2628812 ],
[  0.     ,      0.     ,      1.        ]])
K_inv = np.linalg.inv(K)
P = np.eye(3, 4)
size_to_dist = lambda a: 0.01714102+15.74211494/a

failed_coords = []
movement = []
dists = []
prev_pose = np.arange(5)[1:]
offset = np.array([0.02, 0.08, 0, 0])
prev_time = time.time()


published_msg = ObjectStateMsg()
msg_twist = Twist()
msg_pose = Pose()
published_msg.object_vel = msg_twist
published_msg.object_pos = msg_pose

def print_number_objects(data) :
    print("There are %d objects in this image", data.count)
    rospy.loginfo("There are %d objects in this image", data.count)

def process_bounding_boxes(data):
    print("processing bounding boxes")
    global dists
    global movement
    global failed_coords
    global prev_pose

    for bbox in data.bounding_boxes:
        x1 = bbox.xmin
        y1 = bbox.ymin
        x2 = bbox.xmax
        y2 = bbox.ymax
        c = bbox.id

        if c == 0:
            continue
        if c == 32 or c == 33 or c == 49 or c == 51:
            # trajectory generation
            # x1 = x1.cpu().detach().numpy()
            # y1 = y1.cpu().detach().numpy()
            # x2 = x2.cpu().detach().numpy()
            # y2 = y2.cpu().detach().numpy()
            print("coordinates: ", x1, y1, x2, y2)
            ball_xy = np.array([(x1+x2)/2.0, (y1+y2)/2.0])

            diameter = (abs(x2-x1) + abs(y2-y1))/2.0
            dist = size_to_dist(diameter/2)
            print("dist: ", dist)
            dists += [dist]


            cam_coord = np.dot(K_inv,np.array([[ball_xy[0]],
                                [ball_xy[1]],
                                [1]]))
            # print(cam_coord)


            opti = Opti()
            world_coord = opti.variable(4)
            init_world_coord = prev_pose
            obj = mtimes((cam_coord - mtimes(P, world_coord)).T, (cam_coord - mtimes(P, world_coord)))
            opti.minimize(obj)
            opti.subject_to([pow(world_coord[0], 2) + pow(world_coord[1], 2) + pow(world_coord[2], 2) == dist*dist,
                            world_coord[3] == 1,
                            world_coord[2] >= 0
                            ])
            opti.set_initial(world_coord, init_world_coord)
            ###### CONSTRUCT SOLVER AND SOLVE ######
            opti.solver('ipopt')
            p_opts = {"expand": False, "print_time": False, "ipopt": {"print_level": 0}}
            s_opts = {"max_iter": 1e3}
            opti.solver('ipopt', p_opts, s_opts)
            try:
                sol = opti.solve()
                world_coord_val = sol.value(world_coord)
                movement += [world_coord_val + offset]

                msg_twist = Twist()
                msg_twist.linear.x = (world_coord_val - prev_pose)[0]/(time.time() - prev_time)
                msg_twist.linear.y = (world_coord_val - prev_pose)[1]/(time.time() - prev_time)
                msg_twist.linear.z = (world_coord_val - prev_pose)[2]/(time.time() - prev_time)
                msg_pose = Pose()
                msg_pose.position.x = world_coord_val[0]
                msg_pose.position.y = world_coord_val[1]
                msg_pose.position.z = world_coord_val[2]

                published_msg.object_vel = msg_twist
                published_msg.object_pos = msg_pose

                prev_pose = world_coord_val

                
                # print(world_coord_val)
            except:
                failed_coords += [cam_coord]
                # print(cam_coord)
            # print(world_coord_val/world_coord_val[3])

def display_image(data):
    pass
    







rospy.init_node('process_yolo')
broadcast = rospy.Publisher('obj_states', ObjectStateMsg, queue_size = 1) 
try: 
    print("listening")
    rospy.Subscriber("/darknet_ros/object_detector", Int8, print_number_objects)
    rospy.Subscriber("/darknet_ros/bounding_boxes", BoundingBoxes, process_bounding_boxes)
    rospy.Subscriber("/darknet_ros/detection_image", Image, display_image)
    broadcast.publish(published_msg)
    rospy.spin()
except KeyboardInterrupt:
    print("Ending")

# save trajectory
fig = plt.figure()
ax = plt.axes(projection='3d')
full_data = np.array(movement)
# print(full_data)
# print(dists)
ax.plot3D(full_data[5:,0], full_data[5:,1], full_data[5:,2], 'gray')
ax.set_title('Object Trajectory in 3D Space')
ax.set_xlabel('x')
ax.set_ylabel('y')
ax.set_zlabel('z')
# plt.show()
plt.savefig("trajectory.png")

