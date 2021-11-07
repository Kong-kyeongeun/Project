#include <geometry_msgs/PoseStamped.h>
#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <string>

using namespace std;

class diagnose
{
  private:
  sensor_msgs::NavSatFix gps;          //global 좌표
  geometry_msgs::PoseStamped waypoint; //local 좌표
  ros::NodeHandle nh;
  ros::Publisher waypointPub;
  ros::Subscriber waypointSub;
  ros::Subscriber gpsSub;

  public:
  diagnose()
  {
    waypointPub = nh.advertise<geometry_msgs::PoseStamped>("/diagnose/next_waypoint", 10);
    waypointSub = nh.subscribe<geometry_msgs::PoseStamped>("/local_planner/next_waypoint", 10, &diagnose::wpCallback, this);
    gpsSub = nh.subscribe<sensor_msgs::NavSatFix>("/mavros/global_position/global", 100, &diagnose::gpsCallback, this);
  }
  ~diagnose()
  {
    cout << "Type : diagnose object deleted" << endl;
  }
  void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr &msg)
  {
    gps.latitude = msg->latitude;
    gps.longitude = msg->longitude;
    gps.altitude = msg->altitude;
  }
  void wpCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
  {
    waypoint.pose = msg->pose;
    waypoint.header = msg->header;
    if (failsafe())
    {
      waypointPub.publish(waypoint);
    }
  }
  bool failsafe()
  {
//    if ((36.00 > gps.latitude) || (38.300603 < gps.latitude) || (126.00 > gps.longitude) || (128.830532 < gps.longitude))
//    {
//      ROS_INFO("GPS ERROR");
//      return false;
//    }
    if ((abs(waypoint.pose.position.x) > 500) || (abs(waypoint.pose.position.y) > 500))
    {
      ROS_INFO("WAYPOINT ERROR");
      return false;
    }
    else
      return true;
  }
  void spin()
  {
    ros::Rate r(30);
    while (ros::ok())
    {
      r.sleep();
      ros::spinOnce();
    }
  }
};
int main(int argc, char **argv)
{
  ros::init(argc, argv, "diagnose");
  diagnose *_diagnose = new diagnose();
  _diagnose->spin();
  if (!ros::ok())
  {
    delete _diagnose;
  }
}
