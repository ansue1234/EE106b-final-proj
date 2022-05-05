#!/usr/bin/env/python

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import argparse

"""
Enya, Osher. Adapted from EE106B Project 1.
Set of classes for defining SE(3) trajectories for the spherical obstacle trajectory.
manipulator
"""

# From https://automaticaddison.com/how-to-convert-euler-angles-to-quaternions-using-python/
def get_quaternion_from_euler(roll, pitch, yaw):
    """
    Convert an Euler angle to a quaternion.
    Input
    :param roll: The roll (rotation around x-axis) angle in radians.
    :param pitch: The pitch (rotation around y-axis) angle in radians.
    :param yaw: The yaw (rotation around z-axis) angle in radians.
    Output
    :return qx, qy, qz, qw: The orientation in quaternion [x,y,z,w] format
    """
    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

    return [qx, qy, qz, qw]

class LinearConstantTrajectory:
    def __init__(self, velocity, start_position):
        """
        Remember to call the constructor of Trajectory
        Parameters
        ----------
        ????? You're going to have to fill these in how you see fit
        start x y z

        velocity: np.array (3)
        start_position: np.array (3)
        end_position: np.array (3)
        """
        self.velocity = np.array(velocity)
        self.start_position = np.array(start_position)
    def get_current_pose(self, t):
        # time t (seconds)
        return self.start_position + self.velocity * t
