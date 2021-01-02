#!/usr/bin/env python

import rospy
import traceback 
import numpy as np
from gpiozero import LED
# IMPORT the custom message: 
# we import it "from" the ROS package we created it in (here "mobrob") with an extension of .msg ...
# and actually import the message type by name (here "ME439WheelSpeeds")
from mobrob_util.msg import ME439WheelSpeeds, ImgFrame
from pololu_drv8835_rpi import motors

#scan_msg_centerX = 0
scan_msg_detect = False
blueLED = LED(17)
redLED = LED(27)
buzzer = LED(22)


# =============================================================================
#     Set up a time course of commands
# =============================================================================

# =============================================================================
# # NEW: Determine paths by lines, arcs, pivots and pauses, and create a 
# #  "robot" object to plan it for you. 
# =============================================================================

# Get parameters from rosparam
wheel_width = rospy.get_param('/wheel_width_model') # All you have when planning is a model - you never quite know the truth! 
body_length = rospy.get_param('/body_length')
wheel_diameter = rospy.get_param('/wheel_diameter_model')
wheel_radius = wheel_diameter/2.0
#center = 320
#buffer = 40

####    CODE HERE:
# Create a mobile robot object from the Imported module "me439_mobile_robot_class"
# REMEMBER to call the right file (i.e., use the _HWK if needed)
import me439_mobile_robot_class_v01 as m439rbt
robot = m439rbt.robot(wheel_width, body_length, wheel_radius)

####    CODE HERE:
# Specify stage_settings as a collections of lines, arcs, pivots and pauses
# Use the functions in your Class (imported above as "me439rbt")
# Example: Move Forward and Back, 0.3 meters per second:  
# ** NOTE the signs on the backward driving: backward speed and backward distance! 
stage_settings = np.array( [robot.plan_pause(1.0) ,robot.plan_pivot(-1.0, -np.pi/2),  robot.plan_pause(1.0), robot.plan_line(0.15, 1.5), robot.plan_pause(1.0) ] )
# Example: pause, forward, pause, pivot right 180 deg, pause, return to home, pause, turn, pause. 
# ** NOTE the signs on the Omega and Angle for the "plan_pivot" calls! 
#stage_settings = np.array( [ robot.plan_pause(1.0), robot.plan_line(0.1, 0.3), robot.plan_pause(1.0), robot.plan_pivot(-1.0, -np.pi), robot.plan_pause(1.0), robot.plan_line(0.1, 0.3), robot.plan_pause(1.0), robot.plan_pivot(1.0, np.pi), robot.plan_pause(1.0)] )
####    CODE HERE: ADD YOUR OWN  

#pub_scan_msg = rospy.Publisher('/img_coord', ImgFrame, queue_size=10)
scan_msg = ImgFrame()
# Convert it into a numpy array
stage_settings_array = np.array(stage_settings)
# Convert the first column to a series of times (elapsed from the beginning) at which to switch settings. 
stage_settings_array[:,0] = np.cumsum(stage_settings_array[:,0],0)  # cumsum = "cumulative sum". The last Zero indicates that it should be summed along the first dimension (down a column). 

# =============================================================================
# # END of new section on planning with lines, arcs, pivots and pauses
# =============================================================================
def listener():
    
    
    rospy.Subscriber('/img_pos_raw', ImgFrame,listener_callback)
    while True:
        patrol()
        print("At location.")
        var = True;
        while var:
            print("Detecting...")
            if scan_msg_detect == False:
                rospy.sleep(5)
                print("Detecting...")
                if scan_msg_detect == False:
                    rospy.sleep(5)
                    print("Detecting...")
                    if scan_msg_detect == False:
                        var = False
                        blueLED.off()
                    
                    else:
                        detect()
                        rospy.sleep(5)
                else:
                    detect()
                    rospy.sleep(5)
            else:
                detect()
                rospy.sleep(5)
                
    rospy.spin()
     
def detect():
    print("Person Detected!!!!")
    blueLED.on()
    redLED.off()
    buzzer.on()
    rospy.sleep(0.2)
    redLED.on()
    blueLED.off()
    buzzer.off()
    rospy.sleep(0.2)
    blueLED.on()
    redLED.off()
    buzzer.on()
    rospy.sleep(0.2)
    redLED.on()
    blueLED.off()
    rospy.sleep(0.2)
    buzzer.off()
    blueLED.off()
    redLED.off()
    
def listener_callback(msg_in):
    
    global scan_msg_detect
    scan_msg_detect = msg_in.detect
    
        
#def tracker():
#    
#    center_x = scan_msg_centerX
#
#    buffer_left = center - buffer
#    buffer_right = center + buffer
#
#    if (center_x >= buffer_left and center_x <= buffer_right):
#        print("Centered")
#        pivot("center")
#    
#    elif (center_x < buffer_left):
#        print("left")
#        pivot("left")
#    
#    elif (center_x > buffer_right):
#        print("right")
#        pivot("right")
#        
        
def pivot(dir):
    if dir == "left":
        motors.motor1.setSpeed(0)
        motors.motor2.setSpeed(-92)
        
    elif dir == "right":
        motors.motor1.setSpeed(92)
        motors.motor2.setSpeed(0)
        
    elif dir == "center":
        motors.motor1.setSpeed(0)
        motors.motor2.setSpeed(0)

# Publish desired wheel speeds at the appropriate time. 
def patrol(): 
    # Actually launch a node called "set_desired_wheel_speeds_by_path_specs"
    rospy.init_node('set_desired_wheel_speeds_by_path_specs', anonymous=False)
    print("On the move...")

    # Create the publisher. Name the topic "sensors_data", with message type "Sensors"
    pub_speeds = rospy.Publisher('/wheel_speeds_desired', ME439WheelSpeeds, queue_size=10)
    
   
    # Declare the message that will go on that topic. 
    # Here we use one of the message name types we Imported, and add parentheses to call it as a function. 
    # We could also put data in it right away using . 
    msg_out = ME439WheelSpeeds()
    msg_out.v_left = 0
    msg_out.v_right = 0
    
    # set up a rate basis to keep it on schedule.
    r = rospy.Rate(100) # N Hz
    try: 
        # start a loop 
       t_start = rospy.get_rostime()
        
       while not rospy.is_shutdown():
           future_stages = np.argwhere( stage_settings_array[:,0] >= (rospy.get_rostime()-t_start).to_sec() ) 
           if len(future_stages)>0:
                stage = future_stages[0]
                
           else: 
                break
           msg_out.v_left = stage_settings_array[stage,1]
           msg_out.v_right = stage_settings_array[stage,2]
           # Actually publish the message
           pub_speeds.publish(msg_out)
            # Log the info (optional)
#            rospy.loginfo(pub_speeds)    
            
           r.sleep()
        
        # Here step through the settings. 
#    for (stage in range(0,len(stage_settings_array))):  
#            # Set the desired speeds
#        dur = stage_settings_array[stage,0]
#        msg_out.v_left = stage_settings_array[stage,1]
#        msg_out.v_right = stage_settings_array[stage,2]
#        # Actually publish the message
#        pub_speeds.publish(msg_out)
#        if(stage_settings_array[stage,1] == 0 && stage_settings_array[stage,2] ==0):
#            scan()
#                 
#            
#            # Sleep for just long enough to reach the next command. 
#        sleep_dur = dur - (rospy.get_rostime()-t_start).to_sec()
#        sleep_dur = max(sleep_dur, 0.)  # In case there was a setting with zero duration, we will get a small negative time. Don't use negative time. 
#        rospy.sleep(sleep_dur)
        
    except Exception:
        traceback.print_exc()
        # When done or Errored, Zero the speeds
        msg_out.v_left = 0
        msg_out.v_right = 0
        pub_speeds.publish(msg_out)
        rospy.loginfo(pub_speeds)    
        pass
        
        
    # When done or Errored, Zero the speeds
    msg_out.v_left = 0
    msg_out.v_right = 0
    pub_speeds.publish(msg_out)
    rospy.loginfo(pub_speeds)    


if __name__ == '__main__':
    try:
        listener()
        
    except rospy.ROSInterruptException: 
        pass
