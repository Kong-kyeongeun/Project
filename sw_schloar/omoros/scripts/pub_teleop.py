#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist

twist = Twist()

pub = rospy.Publisher('/cmd_vel', Twist, queue_size=20)

z = 0.
x = 0.

def callback(data):
   z_ = data.angular.z
   x_ = data.linear.x
   global z
   global x

   if z_==0. and x_==0. :
      twist.linear.x = 0

   else :
      z =  z + z_
      x = x + x_

   twist.linear.z = z
   twist.linear.x = x

   pub.publish(twist)
   rospy.loginfo("x : %f" %(x))
   rospy.loginfo("z : %f" %(z))
   # rospy.loginfo("msg : %f" %(twist.linear.x))


if __name__ == '__main__':
   rospy.init_node('twist_pub')
   sub = rospy.Subscriber('/control', Twist, callback)
   # sub = rospy.Subscriber('/stop', Twist, callback)
   # twist.linear.x = 1.5
   rospy.spin()