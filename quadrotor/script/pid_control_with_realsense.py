#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Joy
from nav_msgs.msg import Odometry
from spinal.msg import FlightConfigCmd, FourAxisCommand, UavInfo, PwmInfo, RollPitchYawTerm, RollPitchYawTerms
from geometry_msgs.msg import Twist
import numpy as np
import tf
import math
count = 0
limit_count = 0


class PIDControl():
    def __init__(self):
        rospy.init_node('pidControl')
        rospy.loginfo('activate')
        self.ar_sub = rospy.Subscriber('/camera/odom/sample',Odometry,self.callback)
        self.teleop_sub = rospy.Subscriber('/cmd_vel', Twist, self.TeleopCallback)
        self.ref = np.array([0.0,0.0,0.10,0],dtype = float) #[ref_x,ref_y,ref_z,ref_yaw]
        self.p = np.array([15.0,15.0,400.0,0.2] , dtype = float) #[px,py,pz,pyaw]
        self.i = np.array([1.5,1.5,200.0,0.001], dtype = float) #[ix,iy,iz,iyaw]
        self.d = np.array([4.0,4.0,40.0,0.0], dtype = float) #[dx,dy,dz,dyaw]
        self.error = np.array([0.0,0.0,0.0,0.0], dtype = float) #[error_x, error_y, error_z, error_yaw]
        self.pre_error = np.array([0.0,0.0,0.0,0.0], dtype = float) #[pre_error_x, pre_error_y, pre_error_z, pre_error_yaw]
        self.integral = np.array([0.0,0.0,0.0,0.0], dtype = float) #[integral_x, integral_y, integral_z, integral_yaw]
        self.psi_est = 0
        self.g = 9.8
        self.m = 0.2
        self.motor_num = 4
        self.base_thrust_offset = 1100.0
        self.landing = True
        self.landing_base_thrust = [1100.0]*self.motor_num
        self.flight_cmd_pub = rospy.Publisher('/four_axes/command', FourAxisCommand, queue_size = 1)
        self.motor_cmd_pub = rospy.Publisher('flight_config_cmd', FlightConfigCmd, queue_size = 1)

    def callback(self, msg):
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        euler = tf.transformations.euler_from_quaternion((orientation.x,orientation.y,orientation.z,orientation.w))
        self.psi_est = euler[2]
        self.error = self.ref - np.array([position.x , position.y, position.z, self.psi_est],dtype = float)
    def TeleopCallback(self,msg):
        if msg.linear.z == 0.5: #'t'
            self.landing = False
            # start arming motor
            self.motor_cmd_pub.publish(FlightConfigCmd(FlightConfigCmd.ARM_ON_CMD))
            # send start i control cmd
            rospy.sleep(0.1) 
            self.motor_cmd_pub.publish(FlightConfigCmd(FlightConfigCmd.INTEGRATION_CONTROL_ON_CMD))
        if msg.linear.z == -0.5: #'b'
            self.landing = True

    def main(self):
        r = rospy.Rate(40) # 40hz
        time = rospy.get_time()

        while not rospy.is_shutdown():
            now_time = rospy.get_time()
            # send flight command
            DELTA_T = 1.0/40.0
            self.integral += (self.pre_error + self.error)/2.0 * DELTA_T
            p_term = self.p * self.error
            i_term = self.i * self.integral
            d_term = self.d * (self.error - self.pre_error)/DELTA_T
            accel = p_term + i_term + d_term
            self.pre_error = self.error
            target_angles = np.array( [ (accel[0]*math.sin(self.psi_est)-accel[1]*math.cos(self.psi_est))/self.g,
                                    (accel[0]*math.cos(self.psi_est)+accel[1]*math.sin(self.psi_est))/self.g,
                                    0],
                                    dtype = float) #[phi,theta, psi]
            for i, angle in enumerate(target_angles):
                if angle >= 0.4:
                    target_angles[i] = 0.4
                elif angle <= -0.4:
                    target_angles[i] = -0.4
            if self.error[2] < 0.2 and (now_time - time) > 6:
                #start yaw control
                target_angles[2] = accel[3]
            target_base_thrust = [accel[2]+self.base_thrust_offset]*self.motor_num
            cmd_msg = FourAxisCommand()
            cmd_msg.angles = target_angles
            cmd_msg.base_thrust = target_base_thrust

            if self.landing:
                cmd_msg.base_thrust = self.landing_base_thrust
                # weaken pwm gradually
                self.landing_base_thrust = [self.landing_base_thrust[0]-2.0]*self.motor_num
                # initializing integration
                self.integral = [0.0,0.0,0.0,0.0]
                if self.landing_base_thrust[0] <= 1000:
                    cmd_msg.base_thrust = [0.0]*self.motor_num
                    self.motor_cmd_pub.publish(FlightConfigCmd(FlightConfigCmd.ARM_OFF_CMD))
                    self.motor_cmd_pub.publish(FlightConfigCmd(FlightConfigCmd.INTEGRATION_CONTROL_OFF_CMD))
            else:
                self.landing_base_thrust = target_base_thrust
            self.flight_cmd_pub.publish(cmd_msg)
            #rospy.loginfo("p_term:[%f,%f,%f], i_term: [%f,%f,%f], d_term: [%f,%f,%f]", p_term[0],p_term[1],p_term[2], i_term[0],i_term[1],i_term[2], d_term[0],d_term[1],d_term[2])
            #rospy.loginfo(self.psi_est)
            rospy.loginfo(accel)
            #rospy.loginfo(cmd_msg.base_thrust)
            r.sleep()

if __name__=="__main__":
    try:
        pid = PIDControl();
        pid.main()
    except rospy.ROSInterruptException: pass

