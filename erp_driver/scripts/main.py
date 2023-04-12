#! /usr/bin/env python3

import rospy
from std_msgs.msg import Int32
from time import sleep

rospy.init_node('main',anonymous=True)
steer_pub=rospy.Publisher("/steer",Int32,queue_size=3)
speed_pub=rospy.Publisher("/speed",Int32,queue_size=3)
brake_pub=rospy.Publisher("/brake",Int32,queue_size=3)
gear_pub=rospy.Publisher("/gear",Int32,queue_size=3) #0 전진 #1 중립 #2 후진
def pub(a,b,c,d):
    steer_pub.publish(a)
    speed_pub.publish(b)
    brake_pub.publish(c)
    gear_pub.publish(d)
    print(a)

class Data:
    def __init__(self):
        self.st = 0
        self.st_sub = rospy.Subscriber("/c_steer", Int32, self.steer_callback)

    def steer_callback(self, st_msg):
        self.st = st_msg.data

if __name__ == "__main__":
    st_Data = Data()
    rate = rospy.Rate(1)
    pub(0,0,0,0)
    while not rospy.is_shutdown():
        if st_Data.st >1500 or st_Data.st <-1500:
            pub(st_Data.st,60, 0, 0)
        else:
            pub(st_Data.st , 70, 0 , 0)
        rate.sleep()