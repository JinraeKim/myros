#!/usr/bin/env python3
import sys
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import argparse


bridge = CvBridge()


def talker(image_cv, topic_name):
    pub = rospy.Publisher(topic_name, Image, queue_size=1)
    rospy.init_node('image_publisher', anonymous=True)
    rate = rospy.Rate(30)  # 30hz
    image_msg = bridge.cv2_to_imgmsg(image_cv, "bgr8")
    image_msg.header.frame_id = "spot1/camera_front_color_optical_frame"
    image_msg.header.stamp = rospy.Time.now()
    while not rospy.is_shutdown():
        pub.publish(image_msg)
        rospy.loginfo("Publishing image {}".format(topic_name))
        rate.sleep()

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="Publisher an image file.")
    parser.add_argument("--image", help="Input image file.")
    parser.add_argument("--topic", help="Published image topic", default="/spot1/camera_front/color/image_raw")

    args = parser.parse_args()

    img = cv2.imread(args.image) 

    try:
        talker(img, args.topic)
    except rospy.ROSInterruptException:
        pass
