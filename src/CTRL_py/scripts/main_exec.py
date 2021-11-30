#!/usr/bin/env python

import os
import argparse
import copy
import time
import rospy
import rospkg
import numpy as np
import yaml
import sys
from math import pi
from Ur3Interface import *
from Ur3Scheduler import *
from LaserInterface import *
import matplotlib.pyplot as plt
from math import *

ur3IF = UR3Interface()
ur3SKD = UR3Scheduler(ur3IF)
laserIF = LaserInterface(ur3IF)
# Position for UR3 not blocking the camera
go_away = [270*math.pi/180.0, -90*math.pi/180.0, 90*math.pi/180.0, -90*math.pi/180.0, -90*math.pi/180.0, 225*math.pi/180.0]

# Store world coordinates of sanitizer and the target place
xw_yw_S = [0.3,0.15,0.15,0]
xw_yw_M = [0.3,0.15,0.028,0]
# 20Hz
SPIN_RATE = 20

# UR3 home location
home = [0*math.pi/180.0, 0*math.pi/180.0, 0*math.pi/180.0, 0*math.pi/180.0, 0*math.pi/180.0, 0*math.pi/180.0]

# UR3 current position, using home position for initialization
current_position = copy.deepcopy(home)

thetas = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

digital_in_0 = 0
analog_in_0 = 0.0

suction_on = True
suction_off = False

current_io_0 = False
current_position_set = False

image_shape_define = False

def move_trace(start_pose, discrete_route_set, end_pose, vel, accel):

    ur3SKD.move_along_discrete_trace([[start_pose[0], start_pose[1], start_pose[2] + 0.05, start_pose[3]], start_pose], vel, accel)
    ur3IF.set_gripper(suction_on)

    time.sleep(1.0)
    if (ur3IF.get_gripper_sensor() == False):
        ur3IF.set_gripper(suction_off)
        ur3IF.set_angle(go_away, vel, accel)
        rospy.loginfo("Block Missing, Mission Skipped!")
        return 1
    else:
        ur3SKD.move_along_discrete_trace(discrete_route_set, vel, accel)
        time.sleep(2.0)
    ur3SKD.move_along_discrete_trace([[end_pose[0], end_pose[1], end_pose[2] + 0.05, end_pose[3]], end_pose], vel, accel)
    ur3IF.set_gripper(suction_off)
    time.sleep(1.0)

    error = 0

    return error

def generate_trace():
    raw = laserIF.get_raw_data()
    paths = laserIF.get_extended_path()
    plt.plot(raw[0],raw[1])
    plt.plot(paths[1],paths[2])
    plt.quiver(paths[1], paths[2], paths[3][0], paths[3][1])
    plt.show()
    descrete_trace = []
    count = 0
    for i in range(0,len(paths[1])):
        count = count + 1
        if paths[1][i] < 300 and count > 10 and 100<np.linalg.norm([paths[1][i], paths[2][i]])<350:
            descrete_trace.append([paths[1][i]/1000.0, # x
                                   paths[2][i]/1000.0, # y
                                   paths[0],         # height
                                   atan2(-paths[3][1][i], -paths[3][0][i])/pi*180.0]) # yaw
            count = 0
        descrete_trace.reverse()
    return descrete_trace

"""
Program run from here
"""
def main():

    global go_away
    global xw_yw_S
    global xw_yw_M
    # global variable1
    # global variable2

    # Initialize ROS node
    rospy.init_node('CTRL_py')

    # Check if ROS is ready for operation
    while(rospy.is_shutdown()):
        print("ROS is shutdown!")

    # Initialize the rate to publish to ur3/command
    loop_rate = rospy.Rate(SPIN_RATE)

    vel = 4.0
    accel = 4.0
    print("Picking up Disinfectant Spray")
    ur3IF.set_angle(ur3SKD.inv_kinetics(-0.2,-0.2,0.3,0), vel, accel)
    ur3IF.set_gripper(suction_on)
    ur3IF.set_angle(ur3SKD.inv_kinetics(-0.2,-0.2,0.19,0), vel, accel)    
    time.sleep(1)
    ur3IF.set_angle(ur3SKD.inv_kinetics(-0.2,-0.2,0.3,0), vel, accel)  

    print ("Moving to initial position")
    ur3IF.set_angle(go_away, vel, accel)
    print ("Initial position reached")
    time.sleep(1)
    param_in = raw_input("Press [Enter] when ready:")
    trace = generate_trace()
    ur3SKD.move_along_discrete_trace(trace, 4.0, 4.0)

    rospy.loginfo("Task Completed!")
    print("Use Ctrl+C to exit program")
    rospy.spin()

if __name__ == '__main__':

    try:
        main()
    # When Ctrl+C is executed, it catches the exception
    except rospy.ROSInterruptException:
        pass
