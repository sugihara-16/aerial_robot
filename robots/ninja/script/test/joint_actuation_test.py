#!/usr/bin/env python

from spinal.msg import ServoControlCmd
from sensor_msgs.msg import JointState

import rospy
import numpy as np
import math
import time

class GimbalTest():
    def __init__(self):
        
        self.gimbal_pub = rospy.Publisher('/ninja/joints_ctrl', JointState, queue_size=1)
        self.nav_rate = rospy.Rate(200)

        self.joint_state_cmd_msg = JointState()
        self.joint_state_cmd_msg.name = ['yaw_dock_joint']

        self.t = 0
        self.test_angle_max = 1.5
        
        time.sleep(0.5)

    def main(self):
        flag = False
        while not rospy.is_shutdown():
            self.t = self.t + 0.02
            target_angle = self.test_angle_max * math.sin(self.t)
            if self.t < math.pi * 2:
                self.joint_state_cmd_msg.position = [target_angle]
            else:
                self.joint_state_cmd_msg.position = [0]
            self.gimbal_pub.publish(self.joint_state_cmd_msg)
            self.nav_rate.sleep()

if __name__ == "__main__":

  rospy.init_node("joint_test")

  gimbal_test = GimbalTest()
  gimbal_test.main()
