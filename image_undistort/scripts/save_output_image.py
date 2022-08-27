#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Copyright 2016 Massachusetts Institute of Technology

"""Extract images from a rosbag.
"""

import os
import argparse

import cv2

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class imageSaver:
    def __init__(self, args):
        rospy.init_node('output_sub_node')
        self.bridge = CvBridge()
        self.count = 0
        self.image_count = 0
        self.args = args

        rospy.Subscriber('/output/image', Image, self.cb_image)
        rospy.spin()
    
    def cb_image(self, data):

        
        if self.count % 1 == 0:
            cv_img = self.bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")
            path = os.path.join(self.args.output_dir, "%06i.png" % self.image_count)
            cv2.imwrite(path, cv_img)
            rospy.loginfo('save the image to {}'.format(path))
            self.image_count += 1

        self.count += 1
        


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="Save image from /output/image topic")
    parser.add_argument("output_dir", help="Output directory.")
    args = parser.parse_args()
    saver = imageSaver(args)