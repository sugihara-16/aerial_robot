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
        self.flight_nav_pub = rospy.Publisher('/assemble_quadrotors1/uav/nav',FlightNav , queue_size = 1)
        self.flight_nav_pub_female = rospy.Publisher('/assemble_quadrotors2/uav/nav',FlightNav , queue_size = 1)

    def main(self):
        r = rospy.Rate(40)
        while not rospy.is_shutdown():
            nav_msg_male = FlightNav()
            nav_msg_male.target = 1
            nav_msg_male.control_frame = 1
            nav_msg_male.pos_xy_nav_mode=2
            nav_msg_male.target_pos_x = 0.5
            nav_msg_male.target_pos_y = 0.0
            self.flight_nav_pub.publish(nav_msg_male)
            self.flight_nav_pub_female.publish(nav_msg_male)
            r.sleep()
if __name__=="__main__":
    try:
        dessemble_demo = DessembleDemo();
        dessemble_demo.main()
    except rospy.ROSInterruptException: pass
