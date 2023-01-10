#!/usr/bin/env python
import rospy

from std_msgs.msg import Empty
from std_msgs.msg import Int8
from std_msgs.msg import UInt16
from std_msgs.msg import UInt8
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
import rosgraph
import tf

import sys, select, termios, tty

#!/usr/bin/env python

import sys
import time
import rospy
import math
from std_msgs.msg import Empty, String
from aerial_robot_msgs.msg import FlightNav
from geometry_msgs.msg import PoseStamped

class AssembleJoy():
    def __init__(self):
        rospy.init_node("assemble_joy")
        self.male_robot_ns = "/assemble_quadrotors1"
        self.female_robot_ns = "/assemble_quadrotors2"
        self.male_ns = self.male_robot_ns + "/teleop_command"
        self.female_ns = self.female_robot_ns + "/teleop_command"
        self.first_time_flag = True
        self.arming_flag = False
        self.takeoff_hight = 0.5
        self.male_flight_state = 0
        self.female_flight_state = 0
        self.male_now_pos_x = 0
        self.male_now_pos_y = 0
        self.male_now_pos_z = 0
        self.male_now_yaw = 0
        self.female_now_pos_x =0
        self.female_now_pos_y =0
        self.female_now_pos_z = 0
        self.female_now_yaw = 0
        self.force_landing_time = 0
        self.nav_mode = 2
        self.target_pos_x = 0
        self.target_pos_y = 0
        self.target_vel_x = 0
        self.target_vel_y = 0
        self.target_pos_z = self.takeoff_hight
        self.target_vel_z = 0
        self.target_omega = 0
        self.target_yaw = 0

        self.joy_sub = rospy.Subscriber('joy', Joy, self.joyCallback)
        self.male_mocap_sub = rospy.Subscriber(self.male_robot_ns+'/mocap/pose', PoseStamped, self.malePoseCallback)
        self.female_mocap_sub = rospy.Subscriber(self.female_robot_ns+'/mocap/pose', PoseStamped, self.femalePoseCallback)

        self.male_flight_config_state_sub = rospy.Subscriber(self.male_robot_ns+'/flight_state',UInt8,self.maleStateCallback)
        self.female_flight_config_state_sub = rospy.Subscriber(self.female_robot_ns+'/flight_state',UInt8,self.femaleStateCallback)

        self.male_land_pub = rospy.Publisher(self.male_ns + '/land', Empty, queue_size=1)
        self.male_halt_pub = rospy.Publisher(self.male_ns + '/halt', Empty, queue_size=1)
        self.male_start_pub = rospy.Publisher(self.male_ns + '/start', Empty, queue_size=1)
        self.male_takeoff_pub = rospy.Publisher(self.male_ns + '/takeoff', Empty, queue_size=1)
        self.male_force_landing_pub = rospy.Publisher(self.male_ns + '/force_landing', Empty, queue_size=1)
        self.male_ctrl_mode_pub = rospy.Publisher(self.male_ns + '/ctrl_mode', Int8, queue_size=1)

        self.female_land_pub = rospy.Publisher(self.female_ns + '/land', Empty, queue_size=1)
        self.female_halt_pub = rospy.Publisher(self.female_ns + '/halt', Empty, queue_size=1)
        self.female_start_pub = rospy.Publisher(self.female_ns + '/start', Empty, queue_size=1)
        self.female_takeoff_pub = rospy.Publisher(self.female_ns + '/takeoff', Empty, queue_size=1)
        self.female_force_landing_pub = rospy.Publisher(self.female_ns + '/force_landing', Empty, queue_size=1)
        self.female_ctrl_mode_pub = rospy.Publisher(self.female_ns + '/ctrl_mode', Int8, queue_size=1)

        self.motion_start_pub = rospy.Publisher('assemble_start', Empty, queue_size=1)
        self.motion_stop_pub  = rospy.Publisher('assemble_stop', Empty, queue_size=1)

        self.male_nav_pub = rospy.Publisher(self.male_robot_ns+'/uav/nav', FlightNav, queue_size=10)
        self.female_nav_pub = rospy.Publisher(self.female_robot_ns+'/uav/nav', FlightNav, queue_size=10)
        self.male_hand_pub = rospy.Publisher(self.male_robot_ns+"/hand_command",String , queue_size=10)

    def maleStateCallback(self,msg):
        self.male_flight_state = msg.data
    def femaleStateCallback(self,msg):
        self.female_flight_state = msg.data
    def malePoseCallback(self,msg):
       self.male_now_pos_x= msg.pose.position.x
       self.male_now_pos_y= msg.pose.position.y
       self.male_now_pos_z= msg.pose.position.z
       quaternion = [msg.pose.orientation.x,msg.pose.orientation.y,msg.pose.orientation.z,msg.pose.orientation.w]
       euler = tf.transformations.euler_from_quaternion(quaternion)
       self.male_now_yaw = euler[2]
       self.first_time_flag = False
    def femalePoseCallback(self,msg):
       self.female_now_pos_x= msg.pose.position.x
       self.female_now_pos_y= msg.pose.position.y
       self.female_now_pos_z= msg.pose.position.z
       quaternion = [msg.pose.orientation.x,msg.pose.orientation.y,msg.pose.orientation.z,msg.pose.orientation.w]
       euler = tf.transformations.euler_from_quaternion(quaternion)
       self.female_now_yaw = euler[2]
    def joyCallback(self,msg):
        '''
        <<key allocations>>
        buttons[0]:cross
        buttons[1]:circle
        buttons[2]:triangle
        buttons[3]:square
        buttons[4]:L1
        buttons[5]:R1
        buttons[6]:L2
        buttons[7]:R2
        buttons[8]:SHERE
        buttons[9]:OPTIONS
        buttons[10]:PS
        buttons[11]:L3
        buttons[12]:R3
        axes[0]:L-horizon(left is positive)
        axes[1]:L-vertical(up is positive)
        axes[2]:always 1
        axes[3]:R-horizon(left is positive)
        axes[4]:R-vertical(up is positive)
        axes[5]:always 1
        axes[6]:left is 1, right is -1
        axes[7]:up is 1, down is -1
        '''
        buttons = msg.buttons
        axes = msg.axes
        joy_time = msg.header.stamp.secs
        if(buttons[9]): #OPTIONS:arming
            if(not self.arming_flag):
                rospy.loginfo("ARMING")
                self.male_start_pub.publish(Empty())
                self.female_start_pub.publish(Empty())
                self.arming_flag = True
            else:
                rospy.loginfo("HALT")
                self.male_halt_pub.publish(Empty())
                self.female_halt_pub.publish(Empty())
                self.arming_flag = False

        if(buttons[8]): #SHARE:force landing or halt
            rospy.loginfo("FORCE LANDING")
            self.male_force_landing_pub.publish(Empty())
            self.female_force_landing_pub.publish(Empty())

        if(axes[6] and buttons[1]): #left and circle:takeoff
            rospy.loginfo("TAKEOFF")
            self.target_pos_z = self.takeoff_hight
            self.male_takeoff_pub.publish(Empty())
            self.female_takeoff_pub.publish(Empty())

        if(axes[6] and buttons[3]): #right and square:landing
            rospy.loginfo("LANDING")
            self.target_pos_z = 0.0
            self.male_land_pub.publish(Empty())
            self.female_land_pub.publish(Empty())
        if(axes[7]): #up and down: peg
            rospy.loginfo("PEG")
            if(axes[7]>0):
                self.male_hand_pub.publish("open")
            else:
                self.male_hand_pub.publish("close")
        if(buttons[2]): #triangle:vel_control_mode
            rospy.loginfo("SWITHCED TO VEL_CONTROL")
            self.nav_mode = 1

        if(buttons[0]): #cross:pos_control_mode
            rospy.loginfo("SWITHCED TO POS_CONTROL")
            self.nav_mode = 2
            self.target_pos_x = self.male_now_pos_x
            self.target_pos_y = self.male_now_pos_y
            self.target_pos_z = self.male_now_pos_z
            self.target_yaw = self.male_now_yaw

        if(self.nav_mode == 1):#L-horizon: x velocity, L-vertical: y velocity, R-horizon:yaw
            self.target_vel_y = 0.2*axes[1]
            self.target_vel_x = 0.2*axes[0]
            self.target_pos_z += 0.001*buttons[6] -0.001*buttons[7]
            self.target_yaw += 0.01 * axes[3]

    def main(self):
        while(self.first_time_flag):
            a = 1
        self.target_pos_x = self.male_now_pos_x
        self.target_pos_y = self.male_now_pos_x
        self.target_yaw = self.male_now_yaw
        r = rospy.Rate(40)
        while not rospy.is_shutdown():
            if(self.male_flight_state is 5 and self.female_flight_state is 5):
                #male
                nav_msg_male = FlightNav()
                nav_msg_male.target = 1
                nav_msg_male.control_frame = 1
                nav_msg_male.pos_xy_nav_mode = self.nav_mode
                nav_msg_male.pos_z_nav_mode = 2
                nav_msg_male.yaw_nav_mode = 2
                nav_msg_male.target_pos_x = self.target_pos_x
                nav_msg_male.target_pos_y = self.target_pos_y
                nav_msg_male.target_pos_z = self.target_pos_z
                nav_msg_male.target_yaw = self.target_yaw
                nav_msg_male.target_vel_x = self.target_vel_x
                nav_msg_male.target_vel_y = self.target_vel_y
                #female
                nav_msg_female = FlightNav()
                nav_msg_female.target = 1
                nav_msg_female.control_frame = 1
                nav_msg_female.pos_xy_nav_mode = self.nav_mode
                nav_msg_female.pos_z_nav_mode = 2
                nav_msg_female.yaw_nav_mode = 2
                nav_msg_female.target_pos_x = self.target_pos_x
                nav_msg_female.target_pos_y = self.target_pos_y
                nav_msg_female.target_pos_z = self.target_pos_z
                nav_msg_female.target_yaw = self.target_yaw - 3.14
                nav_msg_female.target_vel_x = -self.target_vel_x
                nav_msg_female.target_vel_y = -self.target_vel_y
                self.male_nav_pub.publish(nav_msg_male)
                self.female_nav_pub.publish(nav_msg_female)
            r.sleep()
if __name__=="__main__":
    try:
        assemble_joy = AssembleJoy();
        assemble_joy.main()
    except rospy.ROSInterruptException: pass
