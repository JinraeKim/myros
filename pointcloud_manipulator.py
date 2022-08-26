#!/usr/bin/env python3
from __future__ import print_function

import sys
import rospy
from sensor_msgs.point_cloud2 import create_cloud, read_points
from sensor_msgs.msg import PointCloud2

import numpy as np


class PointCloudManipulator:
    """
    Highly based on [1].

    Refs:
        [1] https://docs.ros.org/en/api/sensor_msgs/html/point__cloud2_8py_source.html
    """
    def __init__(self):
        self.pcd_pub = rospy.Publisher("/pointcloud", PointCloud2)

        self.pcd_sub = rospy.Subscriber("/camera/depth/color/points", PointCloud2, self.callback)

    def callback(self, pcd):
        _tmp = np.arange(0, 500)
        uvs = [np.array([a, a]) for a in _tmp]  # specific indices
        points_iterator = read_points(pcd, uvs=uvs)
        points = [(point[0], 0, point[2], point[3]) for point in points_iterator]
        pcd_new = create_cloud(pcd.header, pcd.fields, points)
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
