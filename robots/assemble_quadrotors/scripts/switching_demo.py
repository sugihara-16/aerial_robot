#!/usr/bin/env python

import sys
import time
import rospy
import math
from std_msgs.msg import Empty, String
from aerial_robot_msgs.msg import FlightNav
from geometry_msgs.msg import PoseStamped

rospy.init_node("switching_demo")

switching_flag_male_pub = rospy.Publisher("assemble_quadrotors1/switching_flag",Empty , queue_size=10)
switching_flag_female_pub = rospy.Publisher("assemble_quadrotors2/switching_flag", Empty, queue_size=10)

rospy.sleep(0.5)
switching_flag_male_pub.publish()
switching_flag_female_pub.publish()
rospy.sleep(0.5)
