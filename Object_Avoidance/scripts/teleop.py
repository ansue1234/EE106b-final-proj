#!/usr/bin/env python
# from queue import Empty
import numpy as np 
import sys
import rospy
import tf

import sys, select, os
if os.name == 'nt':
    import msvcrt
else:
    import tty, termios
if os.name != 'nt':
    settings = termios.tcgetattr(sys.stdin)
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty


def cmd(data):
    control_cmd = Twist()
    control_cmd.linear.x = data[0]
    control_cmd.linear.y = data[1]
    control_cmd.linear.z= data[2]
    if not(data[0] == data[1] and data[1] == data[2] and data[0] == 0):
        print(data)
    pub_vel.publish(control_cmd)
    
def getKey():
    if os.name == 'nt':
      return msvcrt.getch()

    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def manual():
    rate = rospy.Rate(100)
    data = np.array([ 0.0, 0.0, 0.0])
    while not rospy.is_shutdown():
        key = getKey()
        if key == 'w':
            print("going forward")
            data[0], data[1], data[2] = 0.5, 0, 0
            cmd(data)
            data[0], data[1], data[2] = 0, 0, 0
        elif key == 'a':
            print("going left")
            data[0], data[1], data[2] = 0, 0.5, 0
            cmd(data)
            data[0], data[1], data[2] = 0, 0, 0
        elif key == 'd':
            print("going right")
            data[0], data[1], data[2] = 0, -0.5, 0
            cmd(data)
            data[0], data[1], data[2] = 0, 0, 0
        elif key == 's':
            print("going back")
            data[0], data[1], data[2] = -0.5, 0, 0
            cmd(data)
            data[0], data[1], data[2] = 0, 0, 0
        elif key == 'i':
            print("accending")
            data[0], data[1], data[2] = 0, 0, 0.25
            cmd(data)
            data[0], data[1], data[2] = 0, 0, 0
        elif key == 'k':
            print("descending")
            data[0], data[1], data[2] = 0, 0, -0.25
            cmd(data)
            data[0], data[1], data[2] = 0, 0, 0
        elif key == 'j':
            print("rotating negative radians")
            vel_cmd = Twist()
            vel_cmd.angular.z = -0.2
            pub_vel.publish(vel_cmd)
        elif key == 'l':
            print("rotating positive radians")
            vel_cmd = Twist()
            vel_cmd.angular.z = 0.2
            pub_vel.publish(vel_cmd)
        elif key == ' ':
            print("landing")
            pub_land.publish(Empty())
        elif key == 'e':
            print("Emergency!!!")
            pub_emergency.publish(Empty())
        elif key == 'u':
            print("taking off")
            pub_takeoff.publish(Empty())
        elif key == 'c':
            print("hover")
            data[0], data[1], data[2] = 0, 0, 0
            cmd(data)
        elif (key == '\x03'):
            break
        else:
            data = data
        cmd(data)
        rate.sleep()

if __name__=='__main__':
    
    rospy.init_node('control')
    print("w, a, s, d for lateral movement")
    print("i, k, for up down, j, l, for rotation")
    print("c, for hover, Space for landing, u for take off, e for emergency")
    # rospy.Subscriber('gazebo/model_states',ModelStates,loc_callback)
    pub_vel = rospy.Publisher('/bebop/cmd_vel',Twist,queue_size=10)
    pub_takeoff = rospy.Publisher('/bebop/takeoff',Empty,queue_size=10)
    pub_land = rospy.Publisher('/bebop/land',Empty,queue_size=10)
    pub_emergency = rospy.Publisher('/bebop/reset',Empty,queue_size=10)
    manual()