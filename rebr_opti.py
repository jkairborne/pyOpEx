#!/usr/bin/env python2

"""Class for writing position controller."""

from __future__ import division, print_function, absolute_import

# Import ROS libraries
import roslib
import rospy
import numpy as np
import math
import struct
from math import sin, cos, pi, asin

import mavlink
from pymavlink import mavutil
import serial, string, math, time, calendar
# Import class that computes the desired positions
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import TransformStamped, Twist, TwistStamped, PoseStamped


packet_sequence = 0

def checksum(data, extra): # https://github.com/mavlink/c_library_v1/blob/master/checksum.h
    output = 0xFFFF
    for i in range(len(data)):
        tmp = data[i] ^ (output & 0xFF)
        tmp = (tmp ^ (tmp << 4)) & 0xFF
        output = ((output >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 0xFFFF
    tmp = extra ^ (output & 0xFF)
    tmp = (tmp ^ (tmp << 4)) & 0xFF
    output = ((output >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 0xFFFF
    return output

def message_checksum(msg):
    '''calculate a 8-bit checksum of the key fields of a message, so we
       can detect incompatible XML changes'''
    crc = mavutil.x25crc(msg.name + ' ')
    for f in msg.ordered_fields:
        crc.accumulate(f.type + ' ')
        crc.accumulate(f.name + ' ')
        if f.array_length:
            crc.accumulate(chr(f.array_length))
    return (crc.crc&0xFF) ^ (crc.crc>>8) 



MAV_target_component_id = 1
MAV_MOCAP_extra_crc = 109
MAV_MOCAP_message_id = 138
MAV_system_id = 1
MAV_component_id = 0x54

def send_att_pos_mocap_packet(quat,x,y,z):
    global packet_sequence
    temp = struct.pack('<Q4ffff',
                       0,
                       quat[0],
                       quat[1],
                       quat[2],
                       quat[3],
                       x,
                       y,
                       z)
    temp = struct.pack("<bbbbB36s",
                       36,
                       0,#packet_sequence & 0xFF,
                       MAV_system_id,
                       MAV_component_id,
                       MAV_MOCAP_message_id,
                       temp)
    #print(struct.calcsize(temp))
    print(struct.calcsize("bbbbBQ4ffff"))
    
    temp = struct.pack("<B41sh",
                       0xFE,
                       temp,
                       message_checksum(temp))#struct.unpack("41s",temp), MAV_MOCAP_extra_crc))
    packet_sequence += 1
    return temp


class PositionController(object):
    def __init__(self):
        self.model_name = 'Track1'
        self.ser = serial.Serial('/dev/ttyUSB0',921600)

        # Containers
        self.optitrack_data = PoseStamped() # container to store incoming optitrack data

        # Pblishers and subscribers
        self.sub_optitrack_data = rospy.Subscriber('/vrpn_client_node/{0}/pose'.format(self.model_name),PoseStamped, self.update_optitrack_data) # subscribe to incoming optitrack data

        # TIme tracking variables
        self.startTime = rospy.get_time(); # start time
        self.prevTime = self.startTime; # previous loop time

    def update_optitrack_data(self, actual_optitrack_data): # subscriber to obtain optitrack data
        self.optitrack_data = actual_optitrack_data

    def desired(self):
        # Actual position and orientation
        self.x = self.optitrack_data.pose.position.x
        self.y = self.optitrack_data.pose.position.y
        self.z = self.optitrack_data.pose.position.z
        quaternion = (self.optitrack_data.pose.orientation.x,self.optitrack_data.pose.orientation.y,self.optitrack_data.pose.orientation.z,self.optitrack_data.pose.orientation.w)
        (self.phi,self.theta,self.psi) = euler_from_quaternion(quaternion)
        print("x: %.2f, y: %.2f, z: %.2f, roll: %f, pitch:  %f, yaw:  %f" %(self.x, self.y, self.z,self.phi,self.theta,self.psi) )
        self.ser.write(send_att_pos_mocap_packet(quaternion,self.x,self.y,self.z))
        
        #SEND DATA OVER SERIAL

if __name__ == '__main__':
    rospy.init_node('rebr_opti') # initialize node
    obj = PositionController() # define class object
    rate = rospy.Rate(100) # loop rate

    while not rospy.is_shutdown():
        obj.desired() # call class function through object to publish command velocity 
        rate.sleep() # used to loop at desired rate
    rospy.spin()
