#!/usr/bin/env python

import rospy
# Import "serial" to get data from the AlaMode
import serial   
import traceback 
# Import the message types we will need
from std_msgs.msg import Int32


# Publish sensors data at the rate it comes in
# Here we publish: 
#  Analog value of interest (levels), 
#  and Encoders (counts) 
# Could also publish more Analogs and Ultrasound (microseconds till echo) readings
# Here we publish all in separate topics. 
# In the long-term it would be better to publish them together.  
def sensors_reader(): 
    # Launch a node called "sensors_node"
    rospy.init_node('sensors_node', anonymous=False)

    # Create the publishers. Name each topic "sensors_##", with message type "Int32" 
    # (because that's what is received over the serial port)
    # Note the queue_size=1 statement: don't let it develop a backlog! 
    pub_A0 = rospy.Publisher('/sensors_A0', Int32, queue_size=1)

    pub_E0 = rospy.Publisher('/sensors_E0', Int32, queue_size=1)
    pub_E1 = rospy.Publisher('/sensors_E1', Int32, queue_size=1)
    
    # Declare the message that will go on the topic. 
    # Here we use one of the message name types we Imported, and add parentheses to call it as a function. 
    # We put data in it using the .data field of the message.
    msg_A0 = Int32()
    msg_E0 = Int32()
    msg_E1 = Int32()
    

# Data comes in on the Serial port. Set that up and start it. 

    #----------setup serial--------------
    ser = serial.Serial('/dev/ttyS0')  #serial port to alamode
    ser.baudrate = 115200 
    ser.bytesize = 8
    ser.parity = 'N'
    ser.stopbits = 1
    ser.timeout = 1 # one second time out. 

    ser.flushInput()
    ser.readline()
    # Initialize variables
    tstart = rospy.get_time()


    # MAIN LOOP to keep loading the message with new data. 
    # NOTE that at the moment the data are coming from a separate thread, but this will be replaced with the serial port line reader in the future. 
    while not rospy.is_shutdown():
        newe0 = 0
        newe1 = 0
        newa0 = 0
        new_data_packet = 0
        try: 
            # Here we read the serial port for a string that looks like "e0:123456", which is an Encoder0 reading. 
            # When we get a reading, update the associated motor command
            line = ser.readline().decode().strip() #blocking function, will wait until read entire line
#            print(line)
            line = line.split(":")
            # Element 0 of "line" will be a string that says what the data are: 
            data_type = line[0]
            # Element 1 of "line" will be the value, an Integer
            data_value = int(line[1])
#            print(data_type)
#            print(line)
            if data_type == 'A0':
                msg_A0 = data_value	# Analog reading 
                pub_A0.publish(msg_A0)
            elif data_type == 'E0':    #only use it as an encoder0 reading if it is one. 
                msg_E0 = data_value	# Here is the actual encoder reading. 
                pub_E0.publish(msg_E0)
            elif data_type == 'E1':    #only use it as an encoder1 reading if it is one. 
                msg_E1 = data_value	# Here is the actual encoder reading. 
                pub_E1.publish(msg_E1)
            
        
        except Exception:
            traceback.print_exc()
            pass
            



if __name__ == '__main__':
    try: 
        sensors_reader()
    except rospy.ROSInterruptException: 
        pass
