#! /usr/bin/env python3

from struct import pack
import rospy
from erp_driver.msg import erpStatusMsg, erpCmdMsg

import serial
import numpy as np
from std_msgs.msg import Int32


from ByteHandler import ErpMsg2Packet, Packet2ErpMsg


START_BITS = "535458"

class ERPHandler:
    def __init__(self):
        rospy.init_node("erp_base",anonymous=True)
        
        _port = rospy.get_param("/erp_base/port")
        _baudrate = rospy.get_param("/erp_base/baudrate")
        rospy.loginfo("erp_base::Uart Port : %s", _port)
        rospy.loginfo("erp_base::Baudrate  : %s", _baudrate)
        self.seri = serial.Serial(port=_port, baudrate=_baudrate)

        rospy.loginfo("Serial %s Connected", _port)
        self.alive = 0
        self.packet = erpCmdMsg()
        self.packet.e_stop = False
        self.flag = True

        self.erpMotionMsg_pub = rospy.Publisher(
            "/erp42_status",
            erpStatusMsg,
            queue_size=3
        )
        self.erpCmdMsg_sub = rospy.Subscriber(
            "/erp42_ctrl_cmd",
            erpCmdMsg,
            self.sendPacket
        )
        
        self.st = 0
        self.sp = 0
        self.br = 0
        self.ge = 0

        self.steer_sub = rospy.Subscriber("/steer",Int32,self.steer)
        self.speed_sub=rospy.Subscriber("/speed",Int32,self.speed)
        self.brake_sub = rospy.Subscriber("/brake",Int32,self.brake)
        self.gear_sub=rospy.Subscriber("/gear",Int32,self.gear)
        
    def steer(self, angle_msg):
        self.st = angle_msg.data
        

    def speed(self,speed_msg):
        self.sp=speed_msg.data
        
    def brake(self, brake_msg):
        self.br = brake_msg.data

    def gear(self,gear_msg):
        self.ge=gear_msg.data       

    def recvPacket(self):

        packet = self.seri.read(18)
        # print('recv',packet,'\n')
        if self.flag:
            print("first packet = {0}".format(packet.hex()))
            self.flag = False
        if packet.hex().find(START_BITS) != 0:
            end, data = packet.hex().split(START_BITS)
            packet = bytes.fromhex(START_BITS + data + end)
        self.erpMotionMsg_pub.publish(
            Packet2ErpMsg(packet)
        )
        
    def sendPacket(self, _data):
        self.packet = _data
        
    def serialSend(self, a, b, c, d):
        if a is not None:
            print(a,b,c,d)
            self.packet.steer = int(a)
            self.packet.speed = int(b)
            self.packet.brake = int(c)
            self.packet.gear = int(d)
            packet = ErpMsg2Packet(self.packet, self.alive)
            self.seri.write(packet)

            self.alive += 1
            if self.alive == 256:
                self.alive = 0
        else:
            packet = ErpMsg2Packet(self.packet, self.alive)
            self.seri.write(packet)

            self.alive += 1
            if self.alive == 256:
                self.alive = 0


if __name__ == "__main__":
    ehandler = ERPHandler()
    rate = rospy.Rate(50)
    while not rospy.is_shutdown():
        ehandler.recvPacket()
        ehandler.serialSend(ehandler.st,ehandler.sp,ehandler.br,ehandler.ge)
        rate.sleep()
