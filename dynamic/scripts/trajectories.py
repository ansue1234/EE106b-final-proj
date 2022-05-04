#!/usr/bin/env/python

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import argparse

"""
Enya, Osher.
Set of classes for defining SE(3) trajectories for the end effector of a robot 
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

class Trajectory:

    def __init__(self, total_time):
        """
        Parameters
        ----------
        total_time : float
        	desired duration of the trajectory in seconds 
        """
        self.total_time = total_time

    def target_pose(self, time):
        """
        Returns where the arm end effector should be at time t, in the form of a 
        7D vector [x, y, z, qx, qy, qz, qw]. i.e. the first three entries are 
        the desired end-effector position, and the last four entries are the 
        desired end-effector orientation as a quaternion, all written in the 
        world frame.
        Hint: The end-effector pose with the gripper pointing down corresponds 
        to the quaternion [0, 1, 0, 0]. 
        Parameters
        ----------
        time : float        
    
        Returns
        -------
        7x' :obj:`numpy.ndarray`
            desired configuration in workspace coordinates of the end effector
        """
        pass

    def target_velocity(self, time):
        """
        Returns the end effector's desired body-frame velocity at time t as a 6D
        twist. Note that this needs to be a rigid-body velocity, i.e. a member 
        of se(3) expressed as a 6D vector.
        The function get_g_matrix from utils may be useful to perform some frame
        transformations.
        Parameters
        ----------
        time : float
        Returns
        -------
        6x' :obj:`numpy.ndarray`
            desired body-frame velocity of the end effector
        """
        pass

    def display_trajectory(self, num_waypoints=67, show_animation=False, save_animation=False):
        """
        Displays the evolution of the trajectory's position and body velocity.
        Parameters
        ----------
        num_waypoints : int
            number of waypoints in the trajectory
        show_animation : bool
            if True, displays the animated trajectory
        save_animatioon : bool
            if True, saves a gif of the animated trajectory
        """
        trajectory_name = self.__class__.__name__
        times = np.linspace(0, self.total_time, num=num_waypoints)

        target_positions = np.vstack([self.target_pose(t)[:3] for t in times])
        target_velocities = np.vstack([self.target_velocity(t)[:3] for t in times])
        print(target_velocities)

        fig = plt.figure(figsize=plt.figaspect(0.5))
        colormap = plt.cm.brg(np.fmod(np.linspace(0, 1, num=num_waypoints), 1))

        # Position plot
        ax0 = fig.add_subplot(1, 2, 1, projection='3d')
        pos_padding = [[-0.1, 0.1],
                        [-0.1, 0.1],
                        [-0.1, 0.1]]
        ax0.set_xlim3d([min(target_positions[:, 0]) + pos_padding[0][0], 
                        max(target_positions[:, 0]) + pos_padding[0][1]])
        ax0.set_xlabel('X')
        ax0.set_ylim3d([min(target_positions[:, 1]) + pos_padding[1][0], 
                        max(target_positions[:, 1]) + pos_padding[1][1]])
        ax0.set_ylabel('Y')
        ax0.set_zlim3d([min(target_positions[:, 2]) + pos_padding[2][0], 
                        max(target_positions[:, 2]) + pos_padding[2][1]])
        ax0.set_zlabel('Z')
        ax0.set_title("%s evolution of\nend-effector's position." % trajectory_name)
        line0 = ax0.scatter(target_positions[:, 0], 
                        target_positions[:, 1], 
                        target_positions[:, 2], 
                        c=colormap,
                        s=2)

        # Velocity plot
        ax1 = fig.add_subplot(1, 2, 2, projection='3d')
        vel_padding = [[-0.1, 0.1],
                        [-0.1, 0.1],
                        [-0.1, 0.1]]
        ax1.set_xlim3d([min(target_velocities[:, 0]) + vel_padding[0][0], 
                        max(target_velocities[:, 0]) + vel_padding[0][1]])
        ax1.set_xlabel('X')
        ax1.set_ylim3d([min(target_velocities[:, 1]) + vel_padding[1][0], 
                        max(target_velocities[:, 1]) + vel_padding[1][1]])
        ax1.set_ylabel('Y')
        ax1.set_zlim3d([min(target_velocities[:, 2]) + vel_padding[2][0], 
                        max(target_velocities[:, 2]) + vel_padding[2][1]])
        ax1.set_zlabel('Z')
        ax1.set_title("%s evolution of\nend-effector's translational body-frame velocity." % trajectory_name)
        line1 = ax1.scatter(target_velocities[:, 0], 
                        target_velocities[:, 1], 
                        target_velocities[:, 2], 
                        c=colormap,
                        s=2)

        if show_animation or save_animation:
            def func(num, line):
                line[0]._offsets3d = target_positions[:num].T
                line[0]._facecolors = colormap[:num]
                line[1]._offsets3d = target_velocities[:num].T
                line[1]._facecolors = colormap[:num]
                return line

            # Creating the Animation object
            line_ani = animation.FuncAnimation(fig, func, frames=num_waypoints, 
                                                          fargs=([line0, line1],), 
                                                          interval=max(1, int(1000 * self.total_time / (num_waypoints - 1))), 
                                                          blit=False)
        plt.show()
        if save_animation:
            line_ani.save('%s.gif' % trajectory_name, writer='pillow', fps=60)
            print("Saved animation to %s.gif" % trajectory_name)

class LinearTrajectory(Trajectory):

    def __init__(self, total_time, start_position, end_position):
        """
        Remember to call the constructor of Trajectory
        Parameters
        ----------
        ????? You're going to have to fill these in how you see fit
        start x y z
        start_position: np.array (3)
        end_position: np.array (3)
        """
        pass
        Trajectory.__init__(self, total_time)
        
        self.total_time = total_time
        self.start_position = np.array(start_position)
        self.end_position = np.array(end_position)
        # self.start_x = start_position[0]
        # self.start_y = start_position[1]
        # self.start_z = start_position[2]
        # self.end_x = end_position[0]
        # self.end_y = end_position[1]
        # self.end_z = end_position[2]

    def target_pose(self, time):
        """
        Returns where the arm end effector should be at time t, in the form of a 
        7D vector [x, y, z, qx, qy, qz, qw]. i.e. the first three entries are 
        the desired end-effector position, and the last four entries are the 
        desired end-effector orientation as a quaternion, all written in the 
        world frame.
        Hint: The end-effector pose with the gripper pointing down corresponds 
        to the quaternion [0, 1, 0, 0]. 
        Parameters
        ----------f
        time : float
    
        Returns
        -------
        7x' :obj:`numpy.ndarray`
            desired configuration in workspace coordinates of the end effector
        """
        # star
        w = (2 * np.pi / (2*self.total_time))
        a = (self.start_position - self.end_position) / 2
        b = (self.start_position + self.end_position) / 2
        position = a * np.cos(w * time) + b
        
        """
        
        time = 0 ==> x = start
        time = total_time ==> x = end
        
        a + b = x_start
        b - a = x_end
        b = a + x_end
        a + a + x_end = x_start
        a = (x_start - x_end) / 2
        b = start + end / 2
        a = (start - end)/2
        """
        config = np.ndarray(shape=(7,), buffer=np.array([position[0], position[1], position[2], 0, 1, 0, 0]))
        return config

    def target_velocity(self, time):
        """
        Returns the end effector's desired body-frame velocity at time t as a 6D
        twist. Note that this needs to be a rigid-body velocity, i.e. a member 
        of se(3) expressed as a 6D vector.
        The function get_g_matrix from utils may be useful to perform some frame
        transformations.
        Parameters
        ----------
        time : float
        Returns
        -------
        6x' :obj:`numpy.ndarray`
            desired body-frame velocity of the end effector
        """
        # let start and end velocity be 0
        if time == 0 or time == self.total_time:
            return np.ndarray(shape=(6,), buffer=np.array([0, 0, 0, 0, 0, 0]))

        """
        derivative
        
        x_position = a * np.cos(w * time) + b
        x_vel = -a * w* np.sin(w*time)
        """

        a = (self.start_position - self.end_position) / 2
        b = (self.start_position + self.end_position) / 2
        w = (2 * np.pi / (2*self.total_time))

        velocity = - a * w * np.sin(w * time)
        
        body_frame_velocity = np.ndarray(shape=(6,), buffer=np.array([velocity[0], velocity[1], velocity[2], 0, 0, 0]))
        return body_frame_velocity

class CircularTrajectory(Trajectory):

    def __init__(self, center_position, radius, total_time):
        """
        Remember to call the constructor of Trajectory
        Parameters
        ----------
        ????? You're going to have to fill these in how you see fit
        """
        pass
        Trajectory.__init__(self, total_time)
        self.center_position = center_position
        self.radius = radius
        self.total_time = total_time

    def target_pose(self, time):
        """
        Returns where the arm end effector should be at time t, in the form of a 
        7D vector [x, y, z, qx, qy, qz, qw]. i.e. the first three entries are 
        the desired end-effector position, and the last four entries are the 
        desired end-effector orientation as a quaternion, all written in the 
        world frame.
        Hint: The end-effector pose with the gripper pointing down corresponds 
        to the quaternion [0, 1, 0, 0]. 
        Parameters
        ----------
        time : float        
    
        Returns
        -------
        7x' :obj:`numpy.ndarray`
            desired configuration in workspace coordinates of the end effector
        """
        w = (2 * np.pi / (self.total_time))
        def f(time):
            w = (2 * np.pi / (2*self.total_time))
            start = 0
            end = 2*np.pi
            a = (start - end) / 2
            b = (start + end) / 2
            return a * np.cos(w * time) + b
        theta = f(time)
        # derivative = 0 when time = 0 and time = total_time

        x_position = self.radius * np.cos(theta) + self.center_position[0]
        y_position = self.radius * np.sin(theta) + self.center_position[1]  

        qx, qy, qz, qw = get_quaternion_from_euler(0, 0, theta)
        config = np.ndarray(shape=(7,), buffer=np.array([x_position, y_position, self.center_position[2], qx, qy, qz, qw]))
        return config

    def target_velocity(self, time):
        """
        Returns the end effector's desired body-frame velocity at time t as a 6D
        twist. Note that this needs to be a rigid-body velocity, i.e. a member 
        of se(3) expressed as a 6D vector.
        Parameters
        ----------
        time : float
        Returns
        -------
        6x' :obj:`numpy.ndarray`
            desired body-frame velocity of the end effector
        """

        """
        
        """
        def f(time):
            w = (2 * np.pi / (2*self.total_time))
            start = 0
            end = 2*np.pi
            a = (end - start) / 2
            b = (start + end) / 2
            return a * np.cos(w * time) + b
        
        def df_dt(time):
            w = (2 * np.pi / (2*self.total_time))
            start = 0
            end = 2*np.pi
            a = (end - start) / 2
            b = (start + end) / 2
            return - a * w * np.sin(w * time)


        theta = f(time)
        x_velocity = -self.radius * np.sin(theta) * df_dt(time)
        y_velocity = self.radius * np.cos(theta) * df_dt(time)

        body_frame_velocity = np.ndarray(shape=(6,), buffer=np.array([x_velocity, y_velocity, 0, 0, 0, df_dt(time)]))
        return body_frame_velocity

class PolygonalTrajectory(Trajectory):
    def __init__(self, points, total_time):
        """
        Remember to call the constructor of Trajectory.
        You may wish to reuse other trajectories previously defined in this file.
        Parameters
        ----------
        ????? You're going to have to fill these in how you see fit
        
        points: list of 3-vecs (can be lists)
        (last elem should be same as first elem to make a closed path)
        """
        Trajectory.__init__(self, total_time)
        self.points = [np.array(point) for point in points]
        self.trajectories = [] #[LinearTrajectory(p0, p1) for p0, p1 in zip(points[:-1], points[1:])]

        time = total_time / (len(self.points) - 1)
        self.times = np.asarray([time * i for i in range(0, len(self.points))])

        for p0, p1 in zip(self.points[:-1], self.points[1:]):
            print(p0, p1)
            self.trajectories.append(LinearTrajectory(time, p0, p1))

        self.dbg = 0

    def target_pose(self, time):
        """
        Returns where the arm end effector should be at time t, in the form of a 
        7D vector [x, y, z, qx, qy, qz, qw]. i.e. the first three entries are 
        the desired end-effector position, and the last four entries are the 
        desired end-effector orientation as a quaternion, all written in the 
        world frame.
        Hint: The end-effector pose with the gripper pointing down corresponds 
        to the quaternion [0, 1, 0, 0]. 
        Parameters
        ----------
        time : float        
    
        Returns
        -------
        7x' :obj:`numpy.ndarray`
            desired configuration in workspace coordinates of the end effector
        """

        # find which trajectory to use
        # find index i of times so that times[i] < time < times[i+1]

        # Figures out index of trajectory during time
        # Example [10,20,30] > 5 = [TTT] ... argmax([TTT]) = 0
        # print(time, self.times)
        i = np.argmax(self.times > time) - 1
        if self.dbg % 50 == 0:
            print(i, time - self.times[i], self.times > time)
        self.dbg += 1
        current_trajectory = self.trajectories[i]
        return current_trajectory.target_pose(time - self.times[i])
        # times[:-1] < time, time < times[1:])
    
    def target_velocity(self, time):
        """
        Returns the end effector's desired body-frame velocity at time t as a 6D
        twist. Note that this needs to be a rigid-body velocity, i.e. a member 
        of se(3) expressed as a 6D vector.
        The function get_g_matrix from utils may be useful to perform some frame
        transformations.
        Parameters
        ----------
        time : float
        Returns
        -------
        6x' :obj:`numpy.ndarray`
            desired body-frame velocity of the end effector
        """
        i = np.argmax(self.times > time) - 1
        current_trajectory = self.trajectories[i]
        return current_trajectory.target_velocity(time - self.times[i])

def define_trajectories(args):
    """ Define each type of trajectory with the appropriate parameters."""
    trajectory = None
    if args.task == 'line':
        trajectory = LinearTrajectory(total_time=10, start_position=[0, 10, 5], end_position=[10, 10, 0])
    elif args.task == 'circle':
        trajectory = CircularTrajectory([0, 0, 0], radius=3, total_time=10)
    elif args.task == 'polygon':
        # polygon = [[0, 0, 0], [5, 0, 2], [4, 12, 2], [3, 10, 1], [0, 0, 0]]
        polygon = [[0, 10, 0], [5, 10, 0], [5, 8, 0], [0, 10, 0]]
        trajectory = PolygonalTrajectory(polygon, total_time=20)
    return trajectory

if __name__ == '__main__':
    """
    Run this file to visualize plots of your paths. Note: the provided function
    only visualizes the end effector position, not its orientation. Use the 
    animate function to visualize the full trajectory in a 3D plot.
    """
    parser = argparse.ArgumentParser()
    parser.add_argument('-task', '-t', type=str, default='line', help=
        'Options: line, circle, polygon.  Default: line'
    )
    parser.add_argument('--animate', action='store_true', help=
        'If you set this flag, the animated trajectory will be shown.'
    )
    args = parser.parse_args()

    trajectory = define_trajectories(args)
    
    if trajectory:
        trajectory.display_trajectory(show_animation=args.animate)