#! /usr/bin/env python

import rospy 
import rospkg 
from gazebo_msgs.msg import ModelState 
from gazebo_msgs.srv import GetModelState
from gazebo_msgs.srv import SetModelState
from trajectories import LinearConstantTrajectory
import numpy as np
from time import sleep
import argparse
from get_goal import get_goal_pt, compute_twist


def set_drone_pose_center():
    state_msg = ModelState()
    state_msg.model_name = 'iris_demo'
    state_msg.pose.position.x = 0
    state_msg.pose.position.y = 0
    state_msg.pose.position.z = 0.5

    rospy.wait_for_service('/gazebo/set_model_state')
    try:
        set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        resp = set_state(state_msg)
    except rospy.ServiceException, e:
        print("Service call failed: %s" % e)

def set_drone_pose(trajectory, time, drone_pose):
    state_msg = ModelState()
    dt = 1./60
    state_msg.model_name = 'iris_demo'

    obj_x, obj_y, obj_z  = trajectory.get_current_pose(time)
    # print(obj_x, obj_y, obj_z)

    o_p = np.array([obj_x, obj_y, obj_z])
    o_v = np.array(trajectory.velocity)
    target = get_goal_pt(o_p, o_v, drone_pose)
    target_x, target_y, target_z = target
    print("target pose: ", target_x, target_y, target_z)
    state_msg.pose.position.x = target_x
    state_msg.pose.position.y = target_y
    state_msg.pose.position.z = target_z
    rospy.wait_for_service('/gazebo/set_model_state')
    try:
        set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        resp = set_state(state_msg)
    except rospy.ServiceException, e:
        print("Service call failed: %s" % e)


def set_obstacle_pose(trajectory, time):
    state_msg = ModelState()
    total_time = 10
    state_msg.model_name = 'animated_sphere'

    x, y, z = trajectory.get_current_pose(time)
    state_msg.pose.position.x = x
    state_msg.pose.position.y = y
    state_msg.pose.position.z = z
    rospy.wait_for_service('/gazebo/set_model_state')
    try:
        set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        resp = set_state(state_msg)
    except rospy.ServiceException, e:
        print("Service call failed: %s" % e)

if __name__ == '__main__':
    rospy.init_node('sim')
    try:
        r = rospy.Rate(10)
        trajectory = LinearConstantTrajectory([0.5, -0.5, 0], [-1, 1, 0.5])
        start_time = None
        model_coordinates = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)

        set_drone_pose_center()
        while not rospy.is_shutdown():
            if not start_time:
                start_time = rospy.get_time()
            t = rospy.get_time() - start_time
            resp_coordinates = model_coordinates("iris_demo", "world")
            drone_x = resp_coordinates.pose.position.x
            drone_y = resp_coordinates.pose.position.y
            drone_z = resp_coordinates.pose.position.z
            # drone_x, drone_y, drone_z = resp_coordinates.pose.position
            set_obstacle_pose(trajectory, t)
            set_drone_pose(trajectory, t, np.array([drone_x, drone_y, drone_z]))

    except rospy.ROSInterruptException:
        pass
