#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2

def publish_usb_camera():
    rospy.init_node('usb_camera_publisher', anonymous=True)
    video_device = "/dev/video2"
    cap = cv2.VideoCapture(video_device)
    if not cap.isOpened():
        rospy.logerr("Failed to open USB camera.")
        return
    camera_info = CameraInfo()
    camera_info.width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    camera_info.height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    image_pub = rospy.Publisher('/camera/image_raw', Image, queue_size=10)
    camera_info_pub = rospy.Publisher('/camera/camera_info', CameraInfo, queue_size=10)
    bridge = CvBridge()
    rate = rospy.Rate(30)

    while not rospy.is_shutdown():
        ret, frame = cap.read()
        if ret:
            camera_info.header.stamp = rospy.Time.now()
            camera_info_pub.publish(camera_info)
            image_msg = bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            image_msg.header.stamp = rospy.Time.now()
            image_pub.publish(image_msg)

        rate.sleep()

    cap.release()

if __name__ == '__main__':
    try:
        publish_usb_camera()
    except rospy.ROSInterruptException:
        pass
