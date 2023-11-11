#!/usr/bin/env python

import rospy
from apriltag_ros.msg import AprilTagDetectionArray
from std_msgs.msg import Float64

class aprilBasedNav():
    def __init__(self):            
        rospy.init_node("subpub2")
        # define and initialize member veriants and functions(publisher, subscriber, others)
        self.detectionx = 0.0
        self.pub = rospy.Publisher('detectionx', Float64, queue_size=1)
        self.sub = rospy.Subscriber('tag_detections', AprilTagDetectionArray, self.callback)
        self.rate = rospy.Rate(10)

    def callback(self,data):
        self.detectionx = data.detections[0].pose.pose.pose.position.x
        print(self.detectionx)

    # def subpub():
    #     global detectionx
    #     rospy.init_node('subpub', anonymous = True)
    #     sub = rospy.Subscriber('tag_detections', AprilTagDetectionArray, callback)
    #     pub = rospy.Publisher('detectionx', Float64, queue_size=1)
    #     rate = rospy.Rate(10)

    def main(self):
        while not rospy.is_shutdown():
            self.pub.publish(self.detectionx)
            self.rate.sleep()
            # main process
            # pub.publish(detectionx)
            # rate.sleep()

if __name__ == "__main__":
    try:
        april_base_nav = aprilBasedNav()
        april_base_nav.main()
    except rospy.ROSInterruptException: pass
