#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from aerial_robot_msgs.msg import FlightNav
from aerial_robot_msgs.msg import RollPitchNav
import numpy as np
import tf
import math

class CircleObit():
    def __init__(self):
        rospy.init_node('pidControl')
        rospy.loginfo('activate')
        self.flight_nav_pub = rospy.Publisher('/assemble_quadrotors/uav/nav',FlightNav , queue_size = 1)
        self.roll_pitch_pub = rospy.Publisher('/assemble_quadrotors/uav/roll_pitch_nav',RollPitchNav , queue_size = 1)
        self.r = 0.5
        self.omega = math.radians(12.0)
        self.target_theta = 0
        self.target_x = 0
        self.target_y = 0
        self.target_yaw = 0
        self.target_roll = 0
        self.target_pitch = 0

    def main(self):
        r = rospy.Rate(40) # 40hz
        time = rospy.get_time()
        while not rospy.is_shutdown():
            now_time = rospy.get_time()
            DELTA_T = 1.0/40.0
            nav_msg = FlightNav()
            roll_pitch_msg = RollPitchNav()

            #calculate target position and angles
            self.target_theta = self.target_theta + self.omega*DELTA_T
            self.target_x = math.cos(self.target_theta)*self.r
            self.target_y = math.sin(self.target_theta)*self.r
            self.target_pitch = self.target_x/10.0
            self.target_roll = self.target_y/5.0
            self.target_yaw = self.target_theta + math.pi/2.0
            #set nav_msg's contents
            nav_msg.pos_xy_nav_mode = 2
            nav_msg.yaw_nav_mode = 2
            nav_msg.control_frame = 1
            nav_msg.pos_xy_nav_mode= 2
            nav_msg.target_pos_x = self.target_x
            nav_msg.target_pos_y = self.target_y
            # nav_msg.target_yaw = self.target_yaw
            self.flight_nav_pub.publish(nav_msg)
            #set roll_pitch_msg's contentse
            roll_pitch_msg.target_pitch = self.target_pitch
            roll_pitch_msg.target_roll = self.target_roll
            self.roll_pitch_pub.publish(roll_pitch_msg)
            r.sleep()
if __name__=="__main__":
    try:
        circle_obit = CircleObit();
        circle_obit.main()
    except rospy.ROSInterruptException: pass
