#!/usr/bin/python
import roslib
roslib.load_manifest( 'hrl_rfid' )
import rospy
#import hrl_rfid.ros_M5e_client as rmc

#import time

#rfid_client = rmc.ROS_M5e_Client('Chris_RFID')
#rfid_client.query_mode()
#while not rospy.is_shutdown():
#    time.sleep(0.1)
    
#rfid_client.stop()

from hrl_rfid.ros_M5e import *
ros_rfid = ROS_M5e( name = 'Chris_RFID', readPwr=3000, portStr='/dev/robot/rfidreader', antFuncs=[EleRightEar], callbacks = [] )
ros_rfid.query_mode()
rospy.spin()
