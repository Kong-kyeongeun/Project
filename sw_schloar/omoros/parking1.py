#!/usr/bin/env python
import rospy
import math
import time
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Vector3
import geometry_msgs.msg import Twist

pub = rospy.Publisher('/control', Twist, queue_size = 20) # space bar 
twist = Twist()
obstacle = 0.0
angle = 0
distance = 0.0
degree = 0.0

def callback(data):
    count = int(data.scan_time / data.time_increment)
    for i in range(count):
        global obstacle
        global angle
        if(angle < math.degrees(data.angle_min + data.angle_increment * i) + 180)
            obstacle = data.ranges[i]
            break
    

def parking(data):
    min_range = 15 #최으로 지정
    min_angle
    for i in range(count):
     
        if data.ranges[i] < min_range :
            min_range = data.ranges[i]
            min_angle =  math.degrees(data.angle_min + data.angle_increment * i) + 180

    if min_angle < 180:
        direction = 1
    else:
        direction = 0
'''
def obstacle( a):
    count = int(data.scan_time / data.time_increment)
    for i in range(count):
        if (math.degrees(data.angle_min + data.angle_increment * i) + 180) >= a 
'''

if __name__ == "__main__":
    rospy.init_node('parking1')
    #################### start ################
    twist.linear.x = 1
    pub.publish(twist) 
    angle = 0
    while(obstacle > 0.5)
        sub = rospy.Subscriber('scan', LaserScan, callback)
    twist.linear.x = 0
    pub.publish(twist) 
    ################## detect parking area #####################
    sub = rospy.Subscriber('scan', LaserScan, parking)
    ################ positioning ################
    if direction == 1 : # obstacle right
        twist.angle.z = 1
        pub.publish(twist) 
        time.sleep(3)
        twist.angle.z = 0
        pub.publish(twist) 
    else # obstacle left
        twist.angle.z = -1
        pub.publish(twist) 
        time.sleep(3)
        twist.angle.z = 0
        pub.publish(twist) 
    ################## parking #################
    twist.linear.x = 0.5
    pub.publish(twist)
    while(obstale > 0.3)
        sub = rospy.Subscriber('scan', LaserScan, callback)
    twist.linear.x = 0.0
    pub.publish(twist)
    
