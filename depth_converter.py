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
        # self.depth_pub = rospy.Publisher("/hi", Image)
        # self.depth_pub = rospy.Publisher("/depth_registered/image_rect", Image)

        self.bridge = CvBridge()
        self.depth_sub = rospy.Subscriber("/camera/aligned_depth_to_color/image_raw", Image, self.callback)

    def callback(self, depth_data):
        """
        depth == 0 means no data; see `depth_image_proc` [1, 2]
        Refs:
            [1] https://github.com/ros-perception/image_pipeline/blob/noetic/depth_image_proc/include/depth_image_proc/depth_traits.h#L51
            [2] https://github.com/ros-perception/image_pipeline/blob/noetic/depth_image_proc/include/depth_image_proc/depth_conversions.h#L79
        """
        try:
            cv_depth = self.bridge.imgmsg_to_cv2(depth_data, "16UC1")
        except CvBridgeError as e:
            print(e)

        (rows, cols) = cv_depth.shape
        cv_depth[0:int(rows/2), 0:int(cols/2)] = 0
        # import pdb; pdb.set_trace()
        # cv_depth[:, :] = 1000
        # cv_depth[0:int(rows/2), 0:int(cols/2)] = 1000
        try:
            depth_data_pub = self.bridge.cv2_to_imgmsg(cv_depth, "passthrough")
            depth_data_pub.header = depth_data.header
            self.depth_pub.publish(depth_data_pub)
        except CvBridgeError as e:
            print(e)


def main(args):
    rospy.init_node('image_converter', anonymous=True)
    dc = DepthConverter()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.signal_shutdown("Shutting down")
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)
