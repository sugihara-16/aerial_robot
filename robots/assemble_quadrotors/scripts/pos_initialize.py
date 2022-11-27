#!/usr/bin/env python

import sys
import time
import rospy
import math
from std_msgs.msg import Empty, String
from aerial_robot_msgs.msg import FlightNav
from geometry_msgs.msg import PoseStamped
import numpy as np
import tf

class PosInitialize():
    def __init__(self):

        rospy.init_node("pos_initialize")
        self.male_nav_pub = rospy.Publisher("assemble_quadrotors1/uav/nav", FlightNav, queue_size=10)
        self.female_mocap_sub = rospy.Subscriber('assemble_quadrotors2/mocap/pose', PoseStamped, self.femalePoseCallback)

        # tf listener and broadcaster
        self.listener = tf.TransformListener()
        self.br = tf.TransformBroadcaster()

        self.female_now_pos_x= 0.0
        self.female_now_pos_y= 0.0
        self.female_now_pos_z= 0.0
        self.female_now_yaw = 0.0
    def femalePoseCallback(self,msg):
        self.female_now_pos_x= msg.pose.position.x
        self.female_now_pos_y= msg.pose.position.y
        self.female_now_pos_z= msg.pose.position.z
        quaternion = msg.pose.orientation
        euler = tf.transformations.euler_from_quaternion([quaternion.x,quaternion.y,quaternion.z,quaternion.w])
        self.female_now_yaw = euler[2]

    #main func
    def main(self):
        r = rospy.Rate(40)
        while not rospy.is_shutdown():
            try:
                female_from_world = self.listener.lookupTransform('/world', '/assemble_quadrotors2/root', rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
            #send command
            nav_msg_male = FlightNav()
            nav_msg_male.target = 1
            nav_msg_male.control_frame = 1
            nav_msg_male.pos_xy_nav_mode=2
            nav_msg_male.pos_z_nav_mode=2
            nav_msg_male.yaw_nav_mode = 2
            nav_msg_male.target_pos_x = self.female_now_pos_x - 1.0
            nav_msg_male.target_pos_y = self.female_now_pos_y
            nav_msg_male.target_pos_z = self.female_now_pos_z
            nav_msg_male.target_yaw = self.female_now_yaw - 3.141592
            self.male_nav_pub.publish(nav_msg_male)
            r.sleep()
if __name__=="__main__":
    try:
        pos_initialize = PosInitialize();
        pos_initialize.main()
    except rospy.ROSInterruptException: pass
