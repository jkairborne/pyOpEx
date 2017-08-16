#!/usr/bin/env python

from __future__ import print_function

import time
import roslib
import rospy

from pymavlink import mavutil

from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import TransformStamped, Twist, TwistStamped, PoseStamped

from argparse import ArgumentParser
parser = ArgumentParser(description=__doc__)

parser.add_argument("--baudrate", type=int,
                  help="master port baud rate", default=57600)
parser.add_argument("--device", required=True, help="serial device", default="/dev/ttyACM0")
args = parser.parse_args()

# create a mavlink serial instance


t1 = time.time()

counts = {}



class PositionController(object):
    def __init__(self):
        self.model_name = 'Track1'
        self.master = mavutil.mavlink_connection(args.device, baud=args.baudrate)

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
        self.y = self.optitrack_data.pose.position.z
        self.z = -self.optitrack_data.pose.position.y
        quaternion = (self.optitrack_data.pose.orientation.x,-self.optitrack_data.pose.orientation.z,self.optitrack_data.pose.orientation.y,self.optitrack_data.pose.orientation.w)
        (self.phi,self.theta,self.psi) = euler_from_quaternion(quaternion)
        print("x: %.2f, y: %.2f, z: %.2f, roll: %f, pitch:  %f, yaw:  %f" %(self.x, self.y, self.z,self.phi,self.theta,self.psi) )
        self.master.mav.att_pos_mocap_send(0, quaternion,self.x,self.y,self.z)
        
        #SEND DATA OVER SERIAL

if __name__ == '__main__':
    rospy.init_node('rebr_opti') # initialize node
    obj = PositionController() # define class object
    rate = rospy.Rate(100) # loop rate

    while not rospy.is_shutdown():
        obj.desired() # call class function through object to publish command velocity 
        rate.sleep() # used to loop at desired rate
    rospy.spin()
