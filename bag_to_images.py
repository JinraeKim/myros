#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Copyright 2016 Massachusetts Institute of Technology

import os
import argparse
try:
    from pathlib import Path
except ImportError:
    from pathlib2 import Path  # python 2 backport

import cv2

import rosbag
from cv_bridge import CvBridge


def main():
    """
    Extract a folder of images from a rosbag.
    """
    parser = argparse.ArgumentParser(description="Extract images from a ROS bag.")
    parser.add_argument("bag_file", help="Input ROS bag.")
    # parser.add_argument("image_topic", help="Image topic.")

    args = parser.parse_args()

    bag_file = args.bag_file  # foo.bag
    dir_log = args.bag_file[:-4]  # foo
    
    # topics
    topics_dict = {
        "color": "/camera/color/image_raw",
        "aligned_depth_to_color": "/camera/aligned_depth_to_color/image_raw",
        "depth": "/camera/depth/image_retc_raw",
    }

    # read ROS bag file
    bag = rosbag.Bag(bag_file, "r")
    bridge = CvBridge()
    for k, v in topics_dict.items():
        print("Extract images from %s on %s (no counting if not exists)" % (args.bag_file, k))
        dir_log_specific = dir_log + "/%s" % k
        p = Path(dir_log_specific)
        p.mkdir(exist_ok=True, parents=True,)
        # os.makedirs(dir_log_specific, exist_ok=True)
        count = 0
        for topic, msg, t in bag.read_messages(topics=[v]):
            cv_img = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

            cv2.imwrite(os.path.join(dir_log_specific, "frame%06i.png" % count), cv_img)
            print("Wrote image %i" % count)

            count += 1

    bag.close()

    return

if __name__ == '__main__':
    main()
