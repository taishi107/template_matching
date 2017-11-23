import time
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Int16
from geometry_msgs.msg import Twist

class TEMP():
    def __init__(self):
        self._cv_bridge = CvBridge()
        self._pub = rospy.Publisher("number", Twist, queue_size=1)
        self._sub = rospy.Subscriber('image_topic',Image,self.callback,queue_size=1)
       # cv2.imread()
    def callback(self,image_msg):
        twist = Twist()
        
        img_rgb = self._cv_bridge.imgmsg_to_cv2(image_msg, "bgr8")
        img_gray = cv2.cvtColor(img_rgb, cv2.COLOR_BGR2GRAY)
        number_dist_list=[]
        for i in range(10):
            template = cv2.imread('{}__.png'.format(i),0)
            w, h = template.shape[::-1]
            res = cv2.matchTemplate(img_gray,template,cv2.TM_CCOEFF_NORMED)
            threshold = 0.9
            loc = np.where( res >= threshold)
            for pt in zip(*loc[::-1]):
                numlist = [d for d in number_dist_list if d["number"]==i]
                found = False
                if len(numlist)>0:
                    for nl in numlist:
                        x,y = nl["geometry"]
                        if abs(x-pt[0])<10:
                            found = True
                if not found:
                    number_dist_list.append({"number":i,"geometry":pt,"w":w,"h":h})

            # cv2.imwrite('result.png',img_rgb)
        print "found items: {}".format(len(number_dist_list))
        for nl in number_dist_list:
            pt,w,h = nl["geometry"],nl["w"],nl["h"]
            cv2.rectangle(img_rgb, pt, (pt[0] + w, pt[1] + h), (0,0,255), 2)
        number_sorted_list = sorted(number_dist_list, key=lambda x:x["geometry"][0])
        if len(number_dist_list) != 0:
            number = [d["number"] for d in number_sorted_list]
    
            number_str = map(str,number)
            sensor_value = ''.join(number_str)
            response = int(sensor_value)
      #  self._pub.publish(self._cv_bridge.cv2_to_imgmsg(img_rgb,"bgr8"))
        elif len(number_dist_list) == 0:
            response = 0

        twist.linear.x = response * 0.01
        print twist
        self._pub.publish(twist)

    def main(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('temp_match')
    temp = TEMP()
    temp.main()

