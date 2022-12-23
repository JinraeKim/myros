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
    parser.add_argument("--bag", help="Input ROS bag.")
    parser.add_argument("--camera", help="Camera prefix")

    args = parser.parse_args()

    bag = args.bag  # foo.bag
    camera = args.camera  # foo.bag
    dir_log = args.bag[:-4]  # foo
    
    # topics
    topics_dict = {
        "color": "{}/color/image_raw".format(camera),
        "aligned_depth_to_color": "{}/aligned_depth_to_color/image_raw".format(camera),
        # "points": "{}/depth/color/points".format(camera),  # This is not an image
    }

    # read ROS bag file
    bag = rosbag.Bag(bag, "r")
    bridge = CvBridge()
    for k, v in topics_dict.items():
        print("Extract images from %s on %s (no counting if not exists)" % (args.bag, k))
        dir_log_specific = dir_log + "/%s" % k
        p = Path(dir_log_specific)
        p.mkdir(exist_ok=True, parents=True,)
        # os.makedirs(dir_log_specific, exist_ok=True)
        count = 0
        for topic, msg, t in bag.read_messages(topics=[v]):
            if k == "color":
                desired_encoding = "bgr8"
            elif k == "aligned_depth_to_color":
                desired_encoding = "16UC1"
            else:
                raise ValueError("Invalid encoding")
            cv_img = bridge.imgmsg_to_cv2(msg, desired_encoding=desired_encoding)

            cv2.imwrite(os.path.join(dir_log_specific, "frame%06i.png" % count), cv_img)
            print("Wrote image %i" % count)

            count += 1

    bag.close()

    return

if __name__ == '__main__':
    main()
