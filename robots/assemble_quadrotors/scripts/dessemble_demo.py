#!/usr/bin/env python

import sys
import time
import rospy
import math
from std_msgs.msg import Empty, String
from aerial_robot_msgs.msg import FlightNav
from geometry_msgs.msg import PoseStamped

class DessembleDemo():
    def __init__(self):
        rospy.init_node("dessemble_demo")
        self.switching_flag_male_pub = rospy.Publisher("assemble_quadrotors1/switching_flag",Empty , queue_size=10)
        self.switching_flag_female_pub = rospy.Publisher("assemble_quadrotors2/switching_flag", Empty, queue_size=10)

        self.male_velo_pub = rospy.Publisher("assemble_quadrotors1/uav/nav", FlightNav, queue_size=10)
        self.female_velo_pub = rospy.Publisher("assemble_quadrotors2/uav/nav", FlightNav, queue_size=10)
        self.male_hand_pub = rospy.Publisher("assemble_quadrotors1/hand_command",String , queue_size=10)
        self.female_hand_pub = rospy.Publisher("assemble_quadrotors2/hand_command",String , queue_size=10)
        self.male_mocap_sub = rospy.Subscriber('assemble_quadrotors1/mocap/pose', PoseStamped, self.malePoseCallback)
        self.female_mocap_sub = rospy.Subscriber('assemble_quadrotors2/mocap/pose', PoseStamped, self.femalePoseCallback)

        self.male_now_pos_x = 0.0
        self.male_now_pos_y = 0.0
        self.female_now_pos_x = 0.0
        self.female_now_pos_y = 0.0

    def malePoseCallback(self,msg):
       self.male_now_pos_x= msg.pose.position.x
       self.male_now_pos_y= msg.pose.position.y
    def femalePoseCallback(self,msg):
       self.female_now_pos_x= msg.pose.position.x
       self.female_now_pos_y= msg.pose.position.y

    def main(self):
        # 1: switch to dessemble mode and open the hand
        rospy.sleep(0.5)
        self.male_hand_pub.publish("close") #eject peg
        rospy.sleep(1.5)
        self.female_hand_pub.publish("open") #inactivate magnet
        self.female_hand_pub.publish("open") #inactivate magnet
        self.female_hand_pub.publish("open") #inactivate magnet
        self.switching_flag_male_pub.publish()
        self.switching_flag_female_pub.publish()
        # 2: wait for 0.5s and flight 1m away each other
        rospy.sleep(0.5)
        while(1): # loop to avoid zero_time

            switched_time = rospy.get_time()
            if switched_time !=0.0:
                break
        r = rospy.Rate(40)

        while not rospy.is_shutdown():
            while(1): # loop to avoid zero_time
                now_time = rospy.get_time()
                if now_time !=0.0:
                    break
            elapsed_time = (now_time - switched_time)
            if(elapsed_time < 2.0):
                nav_msg_male = FlightNav()
                nav_msg_male.target = 1
                nav_msg_male.control_frame = 1
                nav_msg_male.pos_xy_nav_mode=1
                nav_msg_male.target_vel_x = -0.1
                nav_msg_female = FlightNav()
                nav_msg_female.target = 1
                nav_msg_female.control_frame = 1
                nav_msg_female.pos_xy_nav_mode=1
                nav_msg_female.target_vel_x = -0.5
                self.male_velo_pub.publish(nav_msg_male)
                self.female_velo_pub.publish(nav_msg_female)
            else:
                #male
                nav_msg_male = FlightNav()
                nav_msg_male.target = 1
                nav_msg_male.control_frame = 1
                nav_msg_male.pos_xy_nav_mode=2
                nav_msg_male.target_pos_x = self.male_now_pos_x
                nav_msg_male.target_pos_y = self.male_now_pos_y
                #female
                nav_msg_female = FlightNav()
                nav_msg_female.target = 1
                nav_msg_female.control_frame = 1
                nav_msg_female.pos_xy_nav_mode=2
                nav_msg_female.target_pos_x = self.female_now_pos_x
                nav_msg_female.target_pos_y = self.female_now_pos_y
                self.male_velo_pub.publish(nav_msg_male)
                self.female_velo_pub.publish(nav_msg_female)
                rospy.loginfo("DESSEMBLE FUNCTION DONE")
                self.male_hand_pub.publish("close")
                break
            r.sleep()
if __name__=="__main__":
    try:
        dessemble_demo = DessembleDemo();
        dessemble_demo.main()
    except rospy.ROSInterruptException: pass
