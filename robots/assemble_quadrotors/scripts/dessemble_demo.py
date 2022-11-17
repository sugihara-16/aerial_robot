#!/usr/bin/env python

import sys
import time
import rospy
import math
from std_msgs.msg import Empty, String
from aerial_robot_msgs.msg import FlightNav


rospy.init_node("dessemble_demo")

switching_flag_male_pub = rospy.Publisher("assemble_quadrotors1/switching_flag",Empty , queue_size=10)
switching_flag_female_pub = rospy.Publisher("assemble_quadrotors2/switching_flag", Empty, queue_size=10)

male_velo_pub = rospy.Publisher("assemble_quadrotors1/uav/nav", FlightNav, queue_size=10)
female_velo_pub = rospy.Publisher("assemble_quadrotors1/uav/nav", FlightNav, queue_size=10)

male_hand_pub = rospy.Publisher("assemble_quadrotors1/hand_command",String , queue_size=10)

# 1: switch to dessemble mode and open the hand
switching_flag_male_pub.publish()
switching_flag_female_pub.publish()
male_hand_pub('open')
# 2: wait for 0.5s and flight 1m away each other
switched_time = rospy.get_time()
r = rospy.Rate(40)
while not rospy.is_shutdown():
    now_time = rospy.get_time()
    elapsed_time = now_time - switched_time
    if(elapsed_time < 2):
        nav_msg_male = FlightNav
        nav_msg_male.target = 1
        nav_msg_male.pos_xy_nav_mode=1
        nav_msg_male.target_vel_x = -0.5
        nav_msg_female = FlightNav
        nav_msg_female.target = 1
        nav_msg_female.pos_xy_nav_mode=1
        nav_msg_female.target_vel_x = -0.5
        male_velo_pub.publish(nav_msg_male)
        female_velo_pub.publish(nav_msg_female)
    else:
        nav_msg_male = FlightNav
        nav_msg_female = FlightNav
        male_velo_pub.publish(nav_msg_male)
        female_velo_pub.publish(nav_msg_female)
        break
    r.sleep()
