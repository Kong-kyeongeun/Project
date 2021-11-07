#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist

twist = Twist()

pub = rospy.Publisher('/cmd_vel', Twist, queue_size=20)

def callback(data):
   twist.linear.x = data.linear.x
   twist.angular.z = data.angular.z
   pub.publish(twist)
   rospy.loginfo("msg : %f" %(twist.linear.x))
   rospy.loginfo("msg : %f" %(twist.angular.z))


if __name__ == '__main__':
   rospy.init_node('twist_pub')
   sub = rospy.Subscriber('/control', Twist, callback)
   rospy.spin()