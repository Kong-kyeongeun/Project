#include <ftm_planning/MR_log.h>
#include <geometry_msgs/PoseStamped.h>
#include <iostream>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/Int8.h>
#include <string>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>

class logger
{

  private:
  ros::NodeHandle nh;
  ros::Publisher logPub;
  ros::Subscriber gpsSub;
  ros::Subscriber modeSub;
  ftm_planning::MR_log log;
  bool IsGNSSSub = false;
  bool IsModeSub = false;

  public:
  logger()
  {
    gpsSub = nh.subscribe<sensor_msgs::NavSatFix>("/mavros/global_position/global", 100, &logger::gpsCallback, this);
    modeSub = nh.subscribe("/MR/planner_mode", 100, &logger::modeCallback, this);
    logPub = nh.advertise<ftm_planning::MR_log>("MR/log", 10);
  }
  ~logger()
  {
  }
  void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr &msg)
  {
    log.gpstime = msg->header.stamp;
    log.latitude = msg->latitude;
    log.longitude = msg->longitude;
    log.altitude = msg->altitude;
    IsGNSSSub = true;
  }
  void modeCallback(const std_msgs::Int8::ConstPtr &msg)
  {
    log.mode = msg->data;
    log.isAutoflight = true;
    IsModeSub = true;
  }
  void spin()
  {
    ros::Rate r(30);
    while (ros::ok())
    {
      if (IsModeSub && IsGNSSSub)
        logPub.publish(log);
      r.sleep();
      ros::spinOnce();
    }
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "setpoint_test");
  //OOP + dynamic allocation adapted.
  logger *_logger = new logger();
  _logger->spin();
  if (!ros::ok())
  {
    delete _logger;
  }
}
