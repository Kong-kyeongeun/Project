#!/usr/bin/env python3

# -*- coding: utf-8 -*-
import rospy
import sys
#import math
#import numpy as np
#from sensor_msgs.msg import NavSatFix
#from geometry_msgs.msg import PoseStamped
from bluetooth import *
from ftm_planning.msg import Mission_arrive
from std_msgs.msg import Int16


arrive_num=0

def MAcallback(data):
    global arrive_num
    print("Mission state: %s", data.data)
    arrive_num = data.data

def connection():
    MA_sub = rospy.Subscriber("mission_planner/bluetooth_mission_request",Int16,MAcallback)
    MR_pub = rospy.Publisher("bluetooth/bluetooth_mission_result",Int16,queue_size=100)
    result=0
    rate=rospy.Rate(1)
    socket = BluetoothSocket( RFCOMM )
    socket.connect(("20:19:07:00:6B:6F", 1))
    socket.settimeout(10)
    print("bluetooth connected!")
    while not rospy.is_shutdown():
        try:
            if(arrive_num == 1):
                msg='1'
            elif(arrive_num == 2):
                msg='2'
            else:
                msg='0'

            socket.send(msg)
            rate.sleep()
            recv_data = socket.recv(1024)
            print("Recived: {}".format(recv_data))
            if(format(recv_data)=="b'1'"):
                result=1
            elif(format(recv_data)=="b'2'"):
                result=2
            MR_pub.publish(result)
        except bluetooth.btcommon.BluetoothError:
           sys.exit()





if __name__ == '__main__':
    try:
        rospy.init_node('bluetoothConnecter',anonymous=True)
        rospy.loginfo("bluetoothConnecter initialized")
        connection()
    except rospy.ROSInterruptException:
        sys.exit()
