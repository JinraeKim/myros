#!/usr/bin/env python3
from __future__ import print_function

import sys
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


class DepthConverter:
    def __init__(self):
        self.depth_pub = rospy.Publisher("/camera/aligned_depth_to_color/image_converted", Image)

        self.bridge = CvBridge()
        self.depth_sub = rospy.Subscriber("/camera/aligned_depth_to_color/image_raw", Image, self.callback)

    def callback(self, depth_data):
        try:
            cv_depth = self.bridge.imgmsg_to_cv2(depth_data, "16UC1")
        except CvBridgeError as e:
            print(e)

        # (rows, cols) = cv_depth.shape
        try:
            self.depth_pub.publish(self.bridge.cv2_to_imgmsg(cv_depth, "16UC1"))
        except CvBridgeError as e:
            print(e)


def main(args):
    dc = DepthConverter()
    rospy.init_node('image_converter', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)
