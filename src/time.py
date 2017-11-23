import rospy
from std_msgs.msg import Float32

class time():
    def __init__(self):
        self.t1_sub = rospy.Subscriber('t1',Float32,self.callt1,queue_size=1)
        self.t2_sub = rospy.Subscriber('t2',Float32,self.callt2,queue_size=1)
        self.t3_sub = rospy.Subscriber('t3',Float32,self.callt3,queue_size=1)
        self.t4_sub = rospy.Subscriber('t4',Float32,self.callt4,queue_size=1)

        #time = self.time1.data + self.time2.data + self.time3.data + self.time4.data
        #print(self.time1.data + self.time2.data + self.time3.data + self.time4.data)
    def callt1(self,t1):
        self.time1 = t1
        #print("t1={0}".format(self.time1.data))
    def callt2(self,t2):
        self.time2 = t2
        #print("t2={0}".format(self.time2.data))
    def callt3(self,t3):
        self.time3 = t3
        #print("t3={0}".format(self.time3.data))
    def callt4(self,t4):
        self.time4 = t4
        #print("t4={0}".format(self.time4.data))
        print("sum_time={0}".format(self.time1.data + self.time2.data + self.time3.data + self.time4.data))

    def main(self):
        #print(self.time1.data + self.time2.data)
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('time')
    a = time()
    a.main()
