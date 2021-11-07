#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist

twist = Twist()

pub = rospy.Publisher('/cmd_vel', Twist, queue_size=20)


def callback(data):
   # twist.angular.z = data.angular.z
   twist.linear.x = data.linear.x
   pub.publish(twist)
   # rospy.loginfo("msg : %f" %(twist.angular.z))
   rospy.loginfo("msg : %f" %(twist.linear.x))


if __name__ == '__main__':
   rospy.init_node('twist_pub')
   #sub = rospy.Subscriber('/control', Twist, callback)
   sub = rospy.Subscriber('/stop', Twist, callback)
   # twist.linear.x = 2.0
   rospy.spin()