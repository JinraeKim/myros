#!/usr/bin/env python3
from __future__ import print_function

import sys
import rospy
from sensor_msgs.point_cloud2 import read_points_list, create_cloud_xyz32
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header


class PointCloudManipulator:
    """
    Highly based on [1].

    Refs:
        [1] https://docs.ros.org/en/api/sensor_msgs/html/point__cloud2_8py_source.html
    """

    def __init__(self):
        self.pcd_pub = rospy.Publisher("/pointcloud", PointCloud2)

        self.pcd_sub = rospy.Subscriber("/camera/depth/color/points", PointCloud2, self.callback)
        self.parent_frame = "/camera_color_optical_frame"

    def callback(self, pcd):
        points = read_points_list(pcd, field_names=["x", "y", "z"])  # Note: this is pretty slow
        header = Header(frame_id=self.parent_frame, stamp=rospy.Time.now())
        pcd_new = create_cloud_xyz32(header, points)
        self.pcd_pub.publish(pcd_new)


def main(args):
    rospy.init_node('point_cloud_reader', anonymous=True)
    pcm = PointCloudManipulator()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")


if __name__ == '__main__':
    main(sys.argv)
