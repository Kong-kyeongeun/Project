#!/usr/bin/env python

import rospy

from geometry_msgs.msg import Twist

twist = Twist()
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=20)
rospy.init_node('pub')
r = rospy.Rate(60)
twist.linear.x=0.5
twist.angular.z=-80/500

while not rospy.is_shutdown():
   pub.publish(twist)
   r.sleep()
