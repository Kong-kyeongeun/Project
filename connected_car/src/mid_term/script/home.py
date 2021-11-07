#!/usr/bin/env python

# -*- coding: utf-8 -*-
import rospy
import math
from mid_term.msg import Topic1

def HomeCallback(data):
    s = cal_arrivetime(data.x,data.y,data.speed)
    h = s//3600
    s = s%3600
    m = s//60
    s = s%60
    print("remaining time")
    print("h :{} m {} s {}".format(h,m,s))

def cal_arrivetime(x,y,speed):
    home_x = rospy.get_param("car_3/home_x");
    home_y = rospy.get_param("car_3/home_y");
    if speed == 0:
        return math.inf
    else:
        distance = ((home_x - x)**2 + (home_y -y)**2)**0.5
        return distance/speed


def spin():
    car_3_sub = rospy.Subscriber("car_3/information",Topic1,HomeCallback)
    home_loc_sub = rospy.Subscriber("car_3/information",Topic1,HomeCallback)
    rate=rospy.Rate(1)
    while not rospy.is_shutdown():
        rate.sleep()


if __name__ == '__main__':
    try:
        rospy.init_node('home',anonymous=True)
        rospy.loginfo("home initialized")
        rospy.set_param("car_3/home_x",50)
        rospy.set_param("car_3/home_y",100)
        spin();
    except rospy.ROSInterruptException:
        pass
