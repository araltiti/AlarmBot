#!/usr/bin/env python

import rospy
# Import "serial" to get data from the AlaMode
import serial   
# Import the message types we will need
from std_msgs.msg import Int32, Float32

# Set up callable Publishers and messages
pub_wheel_speed_desired_left = rospy.Publisher('/wheel_speed_desired_left',Float32,queue_size=1)
pub_wheel_speed_desired_right = rospy.Publisher('/wheel_speed_desired_right',Float32,queue_size=1)



def sensors_to_wheel_speed():
    rospy.init_node('SensorsToMotorCommandNode',anonymous=False)
    
    sub_A0 = rospy.Subscriber('/sensors_A0',Int32,A0_to_motor_command)
    
    # prevent the node from exiting
    rospy.spin()
    
    
def A0_to_motor_command(msg_in):
    
    # unpack the message
    A0 = msg_in.data
    
    #determine the left wheel speed
    wheel_speed_desired_left = 4.0*A0
    if wheel_speed_desired_left > 480.:
        wheel_speed_desired_left = 480.
    if wheel_speed_desired_left < 60.:
        wheel_speed_desired_left = 0.
    # pack and publish
    wheel_speed_desired_left_msg = Float32()
    wheel_speed_desired_left_msg.data = wheel_speed_desired_left
    pub_wheel_speed_desired_left.publish(wheel_speed_desired_left_msg)
    rospy.loginfo("left {0}".format(wheel_speed_desired_left_msg.data)) 
#    rospy.sleep(0.001)
    
    # Dummy code for the right: 
    wheel_speed_desired_right = wheel_speed_desired_left * -1 
    # pack and publish
    wheel_speed_desired_right_msg = Float32()
    wheel_speed_desired_right_msg.data = wheel_speed_desired_right
    pub_wheel_speed_desired_right.publish(wheel_speed_desired_right_msg)
    rospy.loginfo("right {}".format(wheel_speed_desired_right_msg.data))    
#    rospy.sleep(0.001)    
    
# Section to start the execution, with Exception handling.         
if __name__ == "__main__": 
    try: 
        sensors_to_wheel_speed()
    except rospy.ROSInterruptException: 
        pass