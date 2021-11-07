#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import math
import numpy as np
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseStamped
a = 6378137
b = 6356752  # earth length


def CalNPi(x):  # ECEF->ENU
    return (a*a)/math.sqrt(a*a*math.cos(x)*math.cos(x)+b*b*math.sin(x)*math.sin(x))


def ENU_Transform(home_X, home_Y, home_Z, latitude, longitude, altitude):  # 받아온 데이터를 변환
    lat = (latitude*math.pi)/180.0
    lon = (longitude*math.pi)/180.0
    alt = altitude
#    rospy.loginfo("GNSS Cordinate:\n %s,%s,%s",data.latitude,data.longitude,alt)

    X = (CalNPi(lat)+alt)*math.cos(lat)*math.cos(lon)
    Y = (CalNPi(lat)+alt)*math.cos(lat)*math.sin(lon)
    Z = (b*b*CalNPi(lat)/(a*a)+alt)*math.sin(lat)


#    rospy.loginfo("GNSS Cordinate -> ECEF Recieved:\n %s,%s,%s",X,Y,Z)

    E = -math.sin(home_lon_rad)*(X-home_X)+math.cos(home_lon_rad)*(Y-home_Y)
    N = -math.sin(home_lat_rad)*math.cos(home_lon_rad)*(X-home_X)-math.sin(home_lat_rad) * \
        math.sin(home_lon_rad)*(Y-home_Y)+math.cos(home_lat_rad)*(Z-home_Z)
    U = math.cos(home_lat_rad)*math.cos(home_lon_rad)*(X-home_X)+math.cos(home_lat_rad) * \
        math.sin(home_lon_rad)*(Y-home_Y)+math.sin(home_lat_rad)*(Z-home_Z)

    return np.array([E, N, U])
   # rospy.loginfo("ECEF Cordinate -> ENU Recieved:\n %s,%s,%s",E,N,U)


# def cord_transformer(home_X,home_Y,home_Z):


if __name__ == '__main__':
    try:
        rospy.loginfo("cord_transformer initialized")
        rospy.init_node('cord_transformer')
        global home_lat
        global home_lon
        global home_alt
        global wp_lat
        global wp_lon
        global wp_alt
        global mwp_lat
        global mwp_lon
        global mwp_alt
        global frb1_lat
        global frb1_lon
        global frb1_alt
        global frb2_lat
        global frb2_lon
        global frb2_alt
        home_lat = rospy.get_param('/cord_transformer/home_lat', 37.5474622)
        home_lon = rospy.get_param("/cord_transformer/home_lon", 127.0784611)
        home_alt = rospy.get_param("/cord_transformer/home_alt", 51.604283309)
        wp_lat = rospy.get_param("/cord_transformer/wp_lat", 37.5474622)
        wp_lon = rospy.get_param("/cord_transformer/wp_lon", 127.0784611)
        wp_alt = rospy.get_param("/cord_transformer/wp_alt", 61.604288309)
        mwp_lat = rospy.get_param("/cord_transformer/mwp_lat", 37.5477335)
        mwp_lon = rospy.get_param("/cord_transformer/mwp_lon", 127.0787306)
        mwp_alt = rospy.get_param("/cord_transformer/mwp_alt", 61.604288309)
        frb1_lat = rospy.get_param("/cord_transformer/frb1_lat", 37.5476868)
        frb1_lon = rospy.get_param("/cord_transformer/frb1_lon", 127.0784611)
        frb1_alt = rospy.get_param("/cord_transformer/frb1_alt", 61.604283309)
        frb2_lat = rospy.get_param("/cord_transformer/frb2_lat", 37.5481359)
        frb2_lon = rospy.get_param("/cord_transformer/frb2_lon", 127.0784611)
        frb2_alt = rospy.get_param("/cord_transformer/frb2_alt", 61.604283309)

        home_lat_rad = home_lat * math.pi / 180.0
        home_lon_rad = home_lon * math.pi / 180.0
        #wp_lat_rad = wp_lat * math.pi / 180.0
        #wp_lon_rad = wp_lon * math.pi / 180.0
        #mwp_lat_rad = mwp_lat * math.pi / 180.0
        #mwp_lon_rad = mwp_lon * math.pi / 180.0
        home_X = (CalNPi(home_lat_rad)+home_alt) * \
            math.cos(home_lat_rad)*math.cos(home_lon_rad)
        home_Y = (CalNPi(home_lat_rad)+home_alt) * \
            math.cos(home_lat_rad)*math.sin(home_lon_rad)
        home_Z = (b*b*CalNPi(home_lat_rad)/(a*a) +
                  home_alt)*math.sin(home_lat_rad)
        rate = rospy.Rate(1)

        ENU = ENU_Transform(home_X, home_Y, home_Z, wp_lat, wp_lon, wp_alt)
        wp_local = PoseStamped()
        wp_local.pose.position.x = ENU[0]
        wp_local.pose.position.y = ENU[1]
        wp_local.pose.position.z = 20
        TWP_pub = rospy.Publisher(
            '/cord_transformer/wp_point', PoseStamped, queue_size=100)
    #    TWP_pub.publish(wp_local)
        rospy.loginfo("LLH wp Cordinate -> ENU Recieved:\n %s,%s,%s",
                      ENU[0], ENU[1], ENU[2])

        ENU = ENU_Transform(home_X, home_Y, home_Z, mwp_lat, mwp_lon, mwp_alt)
        mwp_local = PoseStamped()
        mwp_local.pose.position.x = ENU[0]
        mwp_local.pose.position.y = ENU[1]
        mwp_local.pose.position.z = 20
        TMWP_pub = rospy.Publisher(
            '/cord_transformer/mwp_point', PoseStamped, queue_size=100)
    #    TMWP_pub.publish(mwp_local)
        rospy.loginfo("LLH wp Cordinate -> ENU Recieved:\n %s,%s,%s",
                      ENU[0], ENU[1], ENU[2])

        ENU = ENU_Transform(home_X, home_Y, home_Z, frb1_lat, frb1_lon, frb1_alt)
        frb1_local = PoseStamped()
        frb1_local.pose.position.x = ENU[0]
        frb1_local.pose.position.y = ENU[1]
        frb1_local.pose.position.z = 20
        TFWP1_pub = rospy.Publisher(
            '/cord_transformer/forb_point1', PoseStamped, queue_size=100)
    #    TFWP_pub.publish(frb1_local)
        rospy.loginfo(
            "LLH mission Cordinate -> ENU Recieved:\n %s,%s,%s", ENU[0], ENU[1], ENU[2]) 

        ENU = ENU_Transform(home_X, home_Y, home_Z, frb2_lat, frb2_lon, frb2_alt)
        frb2_local = PoseStamped()
        frb2_local.pose.position.x = ENU[0]
        frb2_local.pose.position.y = ENU[1]
        frb2_local.pose.position.z = ENU[2]
        TFWP2_pub = rospy.Publisher(
            '/cord_transformer/forb_point2', PoseStamped, queue_size=100)
    #    TFWP_pub.publish(frb2_local)
        rospy.loginfo(
            "LLH mission Cordinate -> ENU Recieved:\n %s,%s,%s", ENU[0], ENU[1], ENU[2])

        # TFRB_pub.publish(np.array([Ef,Nf,Uf]))
        while not rospy.is_shutdown():
            rate.sleep()
            TWP_pub.publish(wp_local)
            TMWP_pub.publish(mwp_local)
            TFWP1_pub.publish(frb1_local)
            TFWP2_pub.publish(frb2_local)
    except rospy.ROSInterruptException:
        pass
