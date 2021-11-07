#include <geometry_msgs/PoseStamped.h>
#include <iostream>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/Int8.h>
#include <string>
#include <tf/transform_broadcaster.h>
#define GROUND 0
#define TAKEOFF 1
#define OFFBOARD 2
#define LAND 3
#define MISSION 4
#define AVOIDANCE 5

using namespace std;

class setpoint_test
{
private:
  sensor_msgs::NavSatFix home_coordinate;
  geometry_msgs::PoseStamped lcl_drone_coordinate;
  geometry_msgs::PoseStamped setpoint;
  ros::NodeHandle nh;
  ros::Publisher targetPub;

  ros::Subscriber gpsSub;
  ros::Subscriber localSub;
  ros::Subscriber waypointSub;
  ros::Subscriber modeSub;
  ros::Subscriber stateSub;
  ros::Subscriber compass_hdgSub;
  ros::Subscriber mavrosModeSub;

  ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
  ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
  ros::ServiceClient take_off_client = nh.serviceClient<mavros_msgs::CommandTOL>("mavros/cmd/takeoff");
  ros::ServiceClient land_client = nh.serviceClient<mavros_msgs::CommandTOL>("mavros/cmd/land");
  int mode = GROUND;
  tf::Transform transform;
  tf::TransformBroadcaster br;
  string mavrosMode;
public:
  setpoint_test()
  {
    targetPub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 100);
    gpsSub = nh.subscribe<sensor_msgs::NavSatFix>("/mavros/global_position/global", 100, &setpoint_test::gpsCallback, this);
    localSub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 100, &setpoint_test::tfCallback, this);
    waypointSub = nh.subscribe<geometry_msgs::PoseStamped>("/g_point", 100, &setpoint_test::wpCallback, this);
    modeSub = nh.subscribe("/local_planner/mode", 100, &setpoint_test::modeCallback, this);
    mavrosModeSub = nh.subscribe<mavros_msgs::State>("/mavros/state", 100, &setpoint_test::stateCallback, this);
  }
  ~setpoint_test()
  {
    cout << "Type : setpoint_test object deleted" << endl;
  }
  void modeCallback(const std_msgs::Int8 &msg)
  {
    switch (msg.data)
    {
    case AVOIDANCE:
    case OFFBOARD:
      //        setOffboard();
      mode = OFFBOARD;

      break;
    case MISSION:
      mode = MISSION;
      break;
    case TAKEOFF:
      if(!(mavrosMode == "stabilized")){
        //          takeoff();
        mode = TAKEOFF;
      }
      break;
    case LAND:
      if(!(mavrosMode == "stabilized")){
        mode = LAND;
//        land();
      }
      break;
    }
  }
  void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr &msg) //home 저장용
  {
    if (mode == GROUND)
    {
      home_coordinate.latitude = msg->latitude;
      home_coordinate.longitude = msg->longitude;
      home_coordinate.altitude = msg->altitude;
    }
  }
  void tfCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
  {
    lcl_drone_coordinate.pose = msg->pose;
    lcl_drone_coordinate.header = msg->header;
  }
  void wpCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
  {
    if((mode == OFFBOARD) || (mode == MISSION)){
      setpoint.pose = msg->pose;
      setpoint.header = msg->header;
      if(setpoint.pose.position.z <= 2.0) return;

      targetPub.publish(setpoint);
//      ROS_INFO("%lf", setpoint.pose.position.z);
    }
  }
  void stateCallback(const mavros_msgs::State::ConstPtr &msg)
  {
    mavrosMode = msg->mode;
  }
  void takeoff()
  {
    mavros_msgs::SetMode takeoff_set_mode;
    takeoff_set_mode.request.base_mode = 0;
    takeoff_set_mode.request.custom_mode = "AUTO.TAKEOFF";
    set_mode_client.call(takeoff_set_mode) && takeoff_set_mode.response.mode_sent;
    mavros_msgs::CommandTOL takeoff_cmd;
    takeoff_cmd.request.altitude = 5;
    takeoff_cmd.request.longitude = home_coordinate.longitude;
    takeoff_cmd.request.latitude = home_coordinate.latitude;
    takeoff_cmd.request.min_pitch = 0;
    tf::Quaternion q;
    q.setX(lcl_drone_coordinate.pose.orientation.x);
    q.setY(lcl_drone_coordinate.pose.orientation.y);
    q.setZ(lcl_drone_coordinate.pose.orientation.z);
    q.setW(lcl_drone_coordinate.pose.orientation.w);
    tf::Matrix3x3 RPY(q);
    double R, P, Y;
    RPY.getRPY(R, P, Y);
    takeoff_cmd.request.yaw = 0;
    (float)Y;
    take_off_client.call(takeoff_cmd);
  }
  void setOffboard()
  {
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.base_mode = 0;
    offb_set_mode.request.custom_mode = "OFFBOARD";
    if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
    {
      ROS_INFO("OFFBOARD enabled");
      mode = OFFBOARD;
    }
    else
    {
      ROS_ERROR("Failed to set OFFBOARD");
    }
  }
  void land()
  {
    mavros_msgs::SetMode land_set_mode;
    land_set_mode.request.base_mode = 0;
    land_set_mode.request.custom_mode = "AUTO.LAND";
    set_mode_client.call(land_set_mode) && land_set_mode.response.mode_sent;
    mavros_msgs::CommandTOL land_cmd;
    land_cmd.request.altitude = 0;
    land_cmd.request.longitude = home_coordinate.longitude;
    land_cmd.request.latitude = home_coordinate.latitude;
    land_cmd.request.min_pitch = 0;
    tf::Quaternion q;
    q.setX(lcl_drone_coordinate.pose.orientation.x);
    q.setY(lcl_drone_coordinate.pose.orientation.y);
    q.setZ(lcl_drone_coordinate.pose.orientation.z);
    q.setW(lcl_drone_coordinate.pose.orientation.w);
    tf::Matrix3x3 RPY(q);
    double R, P, Y;
    RPY.getRPY(R, P, Y);
    land_cmd.request.yaw = 0;
    (float)Y;
    land_client.call(land_set_mode);
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
  ros::init(argc, argv, "setpoint_test");
  //OOP + dynamic allocation adapted.
  setpoint_test *_setpoint_test = new setpoint_test();
  _setpoint_test->spin();
  if (!ros::ok())
  {
    delete _setpoint_test;
  }
}
