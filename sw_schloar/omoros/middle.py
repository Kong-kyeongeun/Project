#!/usr/bin/env python
import rospy
import math
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Twist

distance = 0.0
degree = 0.0
# pub_ = rospy.Publisher('/rplidar_data', Vector3, queue_size = 1000)
# laser_data = Vector3()
pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 20)
twist = Twist()

def callback(data):
    count = int(data.scan_time / data.time_increment)
    global degree
    global distance
    for i in range(count):
        degree = math.degrees(data.angle_min + data.angle_increment * i) + 180
        distance = data.ranges[i]
        # rospy.loginfo("degree : %f, distance : %f"%(degree, distance))

def avoidance():
    while(True):
        if(degree > 270 and degree < 300):
            if(distance < 1.2 and distance > 0.4):
                twist.angular.z = 0.0
                pub.publish(twist)
        elif((degree > 0.0 and degree <= 20.0) or (degree < 360.0 and degree >= 345.0)):
            if (distance < 1.2 and distance > 0.4):
                twist.angular.z = 1.5
                pub.publish(twist)
        else:
            twist.angular.z = -1.5
            pub.publish(twist)
            break

while not rospy.is_shutdown():
    rospy.init_node('object_node')
    sub = rospy.Subscriber('scan', LaserScan, callback)
        
    # rospy.loginfo("degree : %f, distance : %f" %(degree, distance))
        
    if ((degree > 0 and degree <= 20.0) or (degree < 360 and    degree >= 345)):
        rospy.loginfo("degree : %f, distance = %f"%(degree, distance))
        if distance < 1.2 and distance > 0.4:
            rospy.loginfo("distance : %f" %distance)
            twist.linear.x = 0.0
    else:
        twist.linear.x = 2.0

    pub.publish(twist)
    # rospy.loginfo(twist.linear.x)