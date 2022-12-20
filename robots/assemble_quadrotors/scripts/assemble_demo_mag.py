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

class AssembleDemo():
    def __init__(self):

        rospy.init_node("assemble_demo")

        # subscribers & publishers
        self.switching_flag_male_pub = rospy.Publisher("assemble_quadrotors1/switching_flag",Empty , queue_size=10)
        self.switching_flag_female_pub = rospy.Publisher("assemble_quadrotors2/switching_flag", Empty, queue_size=10)

        self.male_nav_pub = rospy.Publisher("assemble_quadrotors1/uav/nav", FlightNav, queue_size=10)
        self.female_nav_pub = rospy.Publisher("assemble_quadrotors2/uav/nav", FlightNav, queue_size=10)
        self.male_hand_pub = rospy.Publisher("assemble_quadrotors1/hand_command",String , queue_size=10)
        self.female_hand_pub = rospy.Publisher("assemble_quadrotors2/hand_command",String , queue_size=10)
        self.start_flag_sub = rospy.Subscriber('/assemble_start',Empty,self.startCallback)
        self.stop_flag_sub = rospy.Subscriber('/assemble_stop',Empty,self.stopCallback)
        self.male_mocap_sub = rospy.Subscriber('assemble_quadrotors1/mocap/pose', PoseStamped, self.malePoseCallback)
        self.female_mocap_sub = rospy.Subscriber('assemble_quadrotors2/mocap/pose', PoseStamped, self.femalePoseCallback)
        # tf listener and broadcaster
        self.listener = tf.TransformListener()
        self.br = tf.TransformBroadcaster()

        # state variables
        self.diff = np.zeros(2 , dtype = float) #male's [x,y,z] from female
        self.diff_pre = np.zeros(2 , dtype = float)
        self.error = np.zeros(2 , dtype = float)
        self.pre_error = np.zeros(2 , dtype = float)
        self.integral = np.zeros(2 , dtype = float)
        self.p = np.zeros(2 , dtype = float) #[px,py]
        self.i = np.zeros(2 , dtype = float) #[ix,iy]
        self.d = np.zeros(2 , dtype = float) #[dx,dy]
        self.start_flag = False
        self.stop_flag = False
        self.male_now_pos_x= 0.0
        self.male_now_pos_y= 0.0
        self.female_now_pos_x= 0.0
        self.female_now_pos_y= 0.0
        self.end_flag = False
        self.close_time = 0.0

        # parameters
        self.p_gain = np.array([0.0,0.01])
        self.i_gain = np.array([0.00,0.06])
        self.d_gain = np.array([0.0,0.08])
        self.y_offset = 0.0
        self.target_diff = np.array([0.42, self.y_offset]) #face to face pos
        self.yaw_from_female = 3.141592 #target male's yaw from female
        self.target_z = 0.0
        self.assemble_thre_1 = 0.45 # CoG diff is 431

    def malePoseCallback(self,msg):
       self.male_now_pos_x= msg.pose.position.x
       self.male_now_pos_y= msg.pose.position.y
    def femalePoseCallback(self,msg):
       self.female_now_pos_x= msg.pose.position.x
       self.female_now_pos_y= msg.pose.position.y
    def startCallback(self,msg):
        self.start_flag = True
        self.stop_flag = False
        self.female_hand_pub.publish("close") #activate magnet
    def stopCallback(self,msg):
        self.start_flag = False
        self.stop_flag = True
        rospy.loginfo("INITIALIZED")
        self.male_hand_pub.publish("close")
        # self.female_hand_pub.publish("open") #inactivate magnet
    #main func
    def main(self):
        r = rospy.Rate(40)
        # 1: open the hand
        rospy.sleep(0.5)
        # 2: Slowly move male closer to the female side
        DELTA_T = 1.0/40.0
        first_time_flag = True
        while not rospy.is_shutdown():
            if(self.end_flag):
                break

            #initialize loop
            while not rospy.is_shutdown():
                if(not self.stop_flag):
                    break
                try:
                    female_from_world = self.listener.lookupTransform('/world', '/assemble_quadrotors2/root', rospy.Time(0))
                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                    continue
                female_pos = female_from_world[0]
                female_euler = tf.transformations.euler_from_quaternion(female_from_world[1])
                #send command
                nav_msg_male = FlightNav()
                nav_msg_male.target = 1
                nav_msg_male.control_frame = 0
                nav_msg_male.pos_xy_nav_mode=2
                nav_msg_male.pos_z_nav_mode=2
                nav_msg_male.yaw_nav_mode = 2
                nav_msg_male.target_pos_x = -1.5
                nav_msg_male.target_pos_y = 0.0
                nav_msg_male.target_pos_z = 1.2
                nav_msg_male.target_yaw = 0
                self.male_nav_pub.publish(nav_msg_male)

                nav_msg_female = FlightNav()
                nav_msg_female.target = 1
                nav_msg_female.control_frame = 0
                nav_msg_female.pos_xy_nav_mode=2
                nav_msg_female.pos_z_nav_mode=2
                nav_msg_female.yaw_nav_mode = 2
                nav_msg_female.target_pos_x = 0.0
                nav_msg_female.target_pos_y = 0.0
                nav_msg_female.target_pos_z = 1.2
                nav_msg_female.target_yaw = 3.14
                self.female_nav_pub.publish(nav_msg_female)
                r.sleep()

            #assemble loop
            while not rospy.is_shutdown():
                if(not self.start_flag):
                    break
                #update each parameters: x,y is acc control & z,yaw is pos control
                try:
                    male_from_female = self.listener.lookupTransform('/assemble_quadrotors2/root', '/assemble_quadrotors1/root', rospy.Time(0))
                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                    continue
                try:
                    female_from_world = self.listener.lookupTransform('/world', '/assemble_quadrotors2/root', rospy.Time(0))
                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                    continue

                male_female_pos = np.array(male_from_female[0])
                self.diff = male_female_pos[:2] # update xy
                self.target_z = male_female_pos[2] # update z
                self.error = self.target_diff - self.diff
                if(first_time_flag):
                    self.pre_error = self.error
                    first_time_flag = False
                self.integral += (self.pre_error + self.error)/2.0 * DELTA_T

                #calculate pid terms & update pre_error
                p_term = self.p_gain * self.error
                i_term = self.i_gain * self.integral
                d_term = self.d_gain * (self.error - self.pre_error)/DELTA_T
                target_acc = p_term + i_term + d_term
                self.pre_error = self.error

                #transform target values to world coordinate
                # female_rot_from_world = np.array(tf.transformations.quaternion_matrix(female_from_world[1]))
                female_euler = tf.transformations.euler_from_quaternion(female_from_world[1])
                female_rot_from_world = np.array(tf.transformations.rotation_matrix((female_euler[2]),(0,0,1)))
                female_rot_from_world_inv = np.linalg.inv(female_rot_from_world)

                transformed_target = np.dot(female_rot_from_world, np.append(target_acc, [0.0,0.0]).reshape(4,1)).reshape(1,4)[0]
                # rospy.loginfo(transformed_target)
                self.br.sendTransform((target_acc[0], target_acc[1], self.target_z),
                                      tf.transformations.quaternion_from_euler(0, 0, self.yaw_from_female),
                                      rospy.Time.now(),
                                      "male_target_values",
                                      "/assemble_quadrotors2/root")
                self.br.sendTransform((0.42, self.y_offset , 0),
                                      tf.transformations.quaternion_from_euler(0, 0, self.yaw_from_female),
                                      rospy.Time.now(),
                                      "male_target_pos",
                                      "/assemble_quadrotors2/root")
                try:
                    homo_transformed_target_acc = self.listener.lookupTransform('/world', '/male_target_values', rospy.Time(0))
                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                    continue
                try:
                    homo_transformed_target_pos = self.listener.lookupTransform('/world', '/male_target_pos', rospy.Time(0))
                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                    continue
                #send command
                if(male_female_pos[0] > self.assemble_thre_1 or abs(male_female_pos[1]) > 0.02):
                    #male
                    nav_msg_male = FlightNav()
                    nav_msg_male.target = 1
                    nav_msg_male.control_frame = 0
                    nav_msg_male.pos_xy_nav_mode=2
                    nav_msg_male.pos_z_nav_mode=2
                    nav_msg_male.yaw_nav_mode=2
                    nav_msg_male.target_acc_y = - homo_transformed_target_acc[0][1]
                    nav_msg_male.target_vel_x = 0.0
                    nav_msg_male.target_vel_y = 0.0
                    nav_msg_male.target_pos_x = homo_transformed_target_pos[0][0]
                    nav_msg_male.target_pos_y = homo_transformed_target_pos[0][1]
                    nav_msg_male.target_pos_z = female_from_world[0][2]
                    nav_msg_male.target_yaw = (tf.transformations.euler_from_quaternion(homo_transformed_target_pos[1]))[2]
                    self.male_nav_pub.publish(nav_msg_male)
                    #female
                    # nav_msg_female = FlightNav()
                    # nav_msg_female.target = 1
                    # nav_msg_female.control_frame = 0
                    # nav_msg_female.pos_xy_nav_mode=1
                    # nav_msg_female.yaw_nav_mode=2
                    # nav_msg_female.target_vel_x = -0.05
                    # nav_msg_female.target_vel_y = 0.0
                    # nav_msg_female.target_yaw = 3.14
                    # self.female_nav_pub.publish(nav_msg_female)
                    self.close_time = 0
                else:
                    rospy.loginfo("activating")
                    #male
                    nav_msg_male = FlightNav()
                    nav_msg_male.target = 1
                    nav_msg_male.control_frame = 1
                    nav_msg_male.pos_xy_nav_mode=1
                    nav_msg_male.target_vel_x = 0.01
                    nav_msg_male.target_vel_y = 0.0
                    self.male_nav_pub.publish(nav_msg_male)
                    #female
                    nav_msg_female = FlightNav()
                    nav_msg_female.target = 1
                    nav_msg_female.control_frame = 1
                    nav_msg_female.pos_xy_nav_mode=1
                    nav_msg_female.target_vel_x = 0.01
                    nav_msg_female.target_vel_y = 0.0
                    self.female_nav_pub.publish(nav_msg_female)
                    if(not self.start_flag):
                        break
                    if(not self.close_time):
                        self.close_time = rospy.get_time()
                    if(self.close_time != 0  and  (rospy.get_time() - self.close_time) > 1.5):
                        self.male_hand_pub.publish("open") #peg insert
                        #male
                        nav_msg_male = FlightNav()
                        nav_msg_male.target =1
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
                        self.male_nav_pub.publish(nav_msg_male)
                        self.female_nav_pub.publish(nav_msg_female)
                        if(not self.start_flag):
                            break
                        self.switching_flag_male_pub.publish()
                        self.switching_flag_female_pub.publish()
                        self.end_flag = True
                        rospy.loginfo("ASSEMBLE DEMO COMPLETED")
                        break
                    r.sleep()

if __name__=="__main__":
    try:
        assemble_demo = AssembleDemo();
        assemble_demo.main()
    except rospy.ROSInterruptException: pass
