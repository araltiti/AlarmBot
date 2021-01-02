#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
title: Motor Node - ME439 Intro to robotics, wisc.edu
"""

import rospy
from std_msgs.msg import Float32 
from pololu_drv8835_rpi import motors, MAX_SPEED  	# MAX_SPEED is 480 (hard-coded)


def listener(): 
    rospy.init_node('motor_node')
    
    # Subscribe to the "wheel_speed_desired_left" topic
    sub = rospy.Subscriber('/wheel_speed_desired_left', Float32, set_wheel_speed_left) 
    
    #### CODE HERE ####
    # Add a Subscriber for the Right wheel
    #### END CODE ####
   
    rospy.spin()    # keep the node from exiting


def set_wheel_speed_left(msg_in): 
    wheel_speed_desired_left = int(msg_in.data)
    motors.motor1.setSpeed(wheel_speed_desired_left)


# Section to start the execution, with Exception handling.     
if __name__ == "__main__":
    try: 
        listener()
    except rospy.ROSInterruptException: 
        motors.motor1.setSpeed(0)
        motors.motor2.setSpeed(0)
        pass