#!/usr/bin/env python

import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

class TEMP(object):
    def __init__(self):
        self._cv_bridge = CvBridge()
        self._pub = rospy.Publisher("hoge", Image, queue_size=1)
        self._sub = rospy.Subscriber("image_topic", Image, self.callback, queue_size=1)

    def callback(self, image_msg):
        img_rgb = self._cv_bridge.imgmsg_to_cv2(image_msg, "bgr8")
        img_gray = cv2.cvtColor(img_rgb, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(img_gray, (5,5),0)
        ret, thresh_img = cv2.threshold(blur, 0,255,cv2.THRESH_BINARY_INV+cv2.THRESH_OTSU)
        self._pub.publish(self._cv_bridge.cv2_to_imgmsg(thresh_img))


    def main(self):
        rospy.spin()

if __name__ == "__main__":
    rospy.init_node("image_match")
    tensor = TEMP()
    tensor.main()
