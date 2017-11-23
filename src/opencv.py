import Image
#import time
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import Int16
from std_msgs.msg import Float32
from cv_bridge import CvBridge

class Ros():
    def __init__(self):
        
        self._cv_bridge = CvBridge()
        self._sub = rospy.Subscriber('/cv_camera/image_raw',Image,self.callback,queue_size=1)
        self._pub = rospy.Publisher('image_topic',Image,queue_size=1)
        #self.time_pub = rospy.Publisher('t1',Float32,queue_size=1)

    def callback(self,image_msg):
        #rate = rospy.Rate(10)
        #start = time.time()
        cv_image = self._cv_bridge.imgmsg_to_cv2(image_msg, "bgr8")
        cv_image_gray = cv2.cvtColor(cv_image,cv2.COLOR_RGB2GRAY)
        ret,cv_image_binary = cv2.threshold(cv_image_gray,100,255,cv2.THRESH_BINARY_INV)
        cv2.imwrite("data.jpg",cv_image)
        #d1 = cv2.imread('data.jpg')
        d1 = cv_image
        height, width, channels = d1.shape

        #height:480 width:640
        clp = d1[100:380, 10:610]
        cv2.imwrite("test-tr.png", clp)

        cv_image = clp
        cv_image_gray = cv2.cvtColor(cv_image,cv2.COLOR_RGB2GRAY)
        ret,cv_image_binary = cv2.threshold(cv_image_gray,100,255,cv2.THRESH_BINARY_INV)
        self._pub.publish(self._cv_bridge.cv2_to_imgmsg(clp,"bgr8"))
        #elapsed_time = time.time() - start
        #self.time_pub.publish(elapsed_time)
        #rate.sleep()
        #print("time:{0}".format(elapsed_time))
    def main(self):
       # rate =rospy.Rate(10)
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('image_capture')
    tensor = Ros()
    tensor.main()
