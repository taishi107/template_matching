import time
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

class TEMP():
    def __init__(self):
        self._cv_bridge = CvBridge()
        self._pub = rospy.Publisher("number", Image, queue_size=1)
        self._sub = rospy.Subscriber('image_topic',Image,self.callback,queue_size=1)
       # cv2.imread()
    def callback(self,image_msg):
        
        img_rgb = self._cv_bridge.imgmsg_to_cv2(image_msg, "bgr8")
        img_gray = cv2.cvtColor(img_rgb, cv2.COLOR_BGR2GRAY)
        template = cv2.imread('6_.png',0)
        w, h = template.shape[::-1]
        res = cv2.matchTemplate(img_gray,template,cv2.TM_CCOEFF_NORMED)
        threshold = 0.83
        loc = np.where( res >= threshold)
        for pt in zip(*loc[::-1]):
            cv2.rectangle(img_rgb, pt, (pt[0] + w, pt[1] + h), (0,0,255), 2)
        # cv2.imwrite('result.png',img_rgb)
        self._pub.publish(self._cv_bridge.cv2_to_imgmsg(img_rgb,"bgr8"))

    def main(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('temp_match')
    temp = TEMP()
    temp.main()

