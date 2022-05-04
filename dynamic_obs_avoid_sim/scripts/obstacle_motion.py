#! /usr/bin/env python

import rospy 
import rospkg 
from gazebo_msgs.msg import ModelState 
from gazebo_msgs.srv import SetModelState
from trajectories import LinearTrajectory, CircularTrajectory, PolygonalTrajectory, define_trajectories
import numpy as np
from time import sleep
import argparse

def set_obstacle_pose(trajectory):
    rospy.init_node('set_obstacle_pose')
    state_msg = ModelState()
    total_time = 10
    dt = 1./60

    state_msg.model_name = 'animated_sphere'

    start_time = rospy.get_time()

    now_from_start = rospy.get_time() - start_time

    for t in np.arange(0 ,trajectory.total_time, dt):
        x, y, z, qx, qy, qz, qw = trajectory.target_pose(t)
        if t % 0.5 == 0:
            print("Pose", x, y, z, qx, qy, qz, qw)
            print("Velocity", trajectory.target_velocity(t))
        state_msg.pose.position.x = x
        state_msg.pose.position.y = y
        state_msg.pose.position.z = z
        state_msg.pose.orientation.x = qx
        state_msg.pose.orientation.y = qy
        state_msg.pose.orientation.z = qz
        state_msg.pose.orientation.w = qw
        rospy.wait_for_service('/gazebo/set_model_state')
        try:
            set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
            resp = set_state( state_msg )
            sleep(dt)
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('-task', '-t', type=str, default='line', help=
        'Options: line, circle, polygon.  Default: line'
    )

    args = parser.parse_args()
    trajectory = define_trajectories(args)
    if trajectory:
        set_obstacle_pose(trajectory)
    

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
