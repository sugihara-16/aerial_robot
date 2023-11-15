#!/usr/bin/env python
import rospy
from apriltag_ros.msg import AprilTagDetectionArray
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
from aerial_robot_msgs.msg import FlightNav
import numpy as np

class aprilPIDcontroller():
    def __init__(self):
        rospy.init_node("aprilpidVel")
        # parameter's for PID control

        #gains
        self.kp = -1.0
        self.ki = -0.005
        # self.ki = 0.00
        self.kd = -0.8

        #duration
        self.dt = 0.025

        #terms
        self.p_term = np.array([0.0, 0.0, 0.0])
        self.i_term = np.array([0.0, 0.0, 0.0])
        self.d_term = np.array([0.0, 0.0, 0.0])
        
        #err, Vel
        self.err_i = np.array([0.0, 0.0, 0.0])
        self.vel = np.array([0.0, 0.0, 0.0])
        self.lastVel = np.array([0.0, 0.0 ,0.0])
        self.error = np.array([0.0, 0.0, 0.0])
        self.pre_error = np.array([0.0, 0.0, 0.0])

        #target distance
        self.targetDistance = np.array([0.0, 0.0, 1.0])
        
        #current distance
        self.currentDistance = np.array([0.0, 0.0, 0.0])

        #ros
        self.pub = rospy.Publisher('/beetle1/uav/nav', FlightNav, queue_size=1)
        self.sub = rospy.Subscriber('tag_detections', AprilTagDetectionArray, self.callback)
        self.rate = rospy.Rate(40)
        self.nav_msg = FlightNav()

        #flags
        self.tag_lost_flag = False

    def calculateError(self):
        self.error = self.targetDistance - self.currentDistance
        self.err_i += self.error
        self.pre_error = self.error
        
    def callback(self,data):
        if(data):
            hogehoge
            self.tag_lost_flag = False
        else:
            self.tag_lost_flag = True
            hogehoge
                
        #set values to current pose
        position =  data.detections[0].pose.pose.pose.position
        # x = -y, y = -x, z = -z
        self.currentDistance = [-position.y, -position.x, position.z]
        
    def main(self):
        while not rospy.is_shutdown():
            #culc error
            self.calculateError()
            
            # calcurate terms
            self.p_term = self.kp * self.error
            self.i_term = self.ki * self.err_i
            self.d_term = self.kd * (self.error - self.pre_error) / self.dt
            
            # calculate main process
            self.vel = self.p_term + self.i_term + self.d_term
            
            self.nav_msg.control_frame = 1
            self.nav_msg.target = 1
            self.nav_msg.pos_xy_nav_mode = 1
                
            self.nav_msg.target_vel_x = self.vel[0]
            self.nav_msg.target_vel_y = self.vel[1]
            self.nav_msg.pos_z_nav_mode = 1
            self.nav_msg.target_vel_z = self.vel[2]
            self.pub.publish(self.nav_msg)

            rospy.loginfo(self.vel)
            self.rate.sleep()
            
if __name__ == "__main__":
    try:
        aprilpid = aprilPIDcontroller()
        aprilpid.main()
    except rospy.ROSInterruptException: pass
