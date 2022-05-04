#! /usr/bin/env python

import rospy 
import rospkg 
from gazebo_msgs.msg import ModelState 
from gazebo_msgs.srv import SetModelState
from trajectories import LinearTrajectory, CircularTrajectory, PolygonalTrajectory
import numpy as np
import actionlib

def main():
    rospy.init_node('set_pose')

    state_msg = ModelState()
    total_time = 10
    dt = 1./60
    lin_trajectory = LinearTrajectory(total_time=total_time, start_position=[-5, 5, 0], end_position=[5, -5, 0])
    
    print(np.arange(0 ,lin_trajectory.total_time, dt))
    state_msg.model_name = 'animated_sphere'

    start_time = rospy.get_time()

    now_from_start = rospy.get_time() - start_time

    for t in np.arange(0 ,lin_trajectory.total_time, dt):
    
    while ()
        x, y, z, qx, qy, qz, qw = lin_trajectory.target_pose(t)
        print(x, y, z, qx, qy, qz, qw)
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

    except rospy.ServiceException, e:
        print "Service call failed: %s" % e

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
