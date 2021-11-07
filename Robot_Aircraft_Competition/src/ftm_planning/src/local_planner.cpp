#include <ftm_planning/Mission_arrive.h>
#include <geometry_msgs/PoseStamped.h>
#include <iostream>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h> //for compass_hdg
#include <std_msgs/Int8.h>
#include <string>
#define GROUND 0
#define TAKEOFF 1
#define OFFBOARD 2
#define LAND 3
#define MISSION 4
#define AVOIDANCE 5

using namespace std;

class local_planner
{
private:
  ros::NodeHandle nh;
  geometry_msgs::PoseStamped next_waypoint;
  geometry_msgs::PoseStamped waypoint;
  geometry_msgs::PoseStamped missionwayPoints;
  geometry_msgs::PoseStamped lcl_drone_coordinate;

  geometry_msgs::PoseStamped forbiddenCenter;
  geometry_msgs::PoseStamped forbiddenCenter1;
  geometry_msgs::PoseStamped forbiddenCenter2;

  sensor_msgs::NavSatFix home_coordinate;


  std_msgs::Int8 mode_msg;
  std_msgs::Int8 r;
  ftm_planning::Mission_arrive mission_arrive;
  ros::Publisher next_wpPub; //TO diagnose
  ros::Publisher modePub;    //TO setpoint_test
  ros::Publisher arrivePub;
  ros::Publisher r_Pub;
  ros::Publisher fb_pub;
  ros::Publisher target_pub;
  ros::Subscriber fb_sub1;
  ros::Subscriber fb_sub2;

  ros::Subscriber localSub;
  ros::Subscriber wpSub;
  ros::Subscriber mwpSub;
  ros::Subscriber YawSub;
  ros::Subscriber stateSub;
  ros::Subscriber mission_resultSub;
  ros::Subscriber gpsSub;
  ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
  ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
  ros::ServiceClient take_off_client = nh.serviceClient<mavros_msgs::CommandTOL>("mavros/cmd/takeoff");
  ros::ServiceClient land_client = nh.serviceClient<mavros_msgs::CommandTOL>("mavros/cmd/land");
  bool wparrive = false;
  bool ARM = false;
  bool mission_result = false;
  int mode = GROUND;
  double flight_level = 20;
  int radius1 = nh.param("/local_planner/radius1",24);

  int radius2 = nh.param("/local_planner/radius2",0);

  int takeoff_stage = 0;

  bool _global_home_exist = false;

  mavros_msgs::State state;
  int land_stage = 1;


public:
  local_planner()
  {
    fb_sub1 = nh.subscribe<geometry_msgs::PoseStamped>("/cord_transformer/forb_point1",100,&local_planner::fb1Callback,this);
    fb_sub2 = nh.subscribe<geometry_msgs::PoseStamped>("/cord_transformer/forb_point2",100,&local_planner::fb2Callback,this);
    fb_pub = nh.advertise<geometry_msgs::PoseStamped>("/local_planner/forb_point", 100);
    r_Pub = nh.advertise<std_msgs::Int8>("/local_planner/radius", 100);

    target_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 100);

    next_wpPub = nh.advertise<geometry_msgs::PoseStamped>("/local_planner/next_waypoint", 100);
    modePub = nh.advertise<std_msgs::Int8>("/local_planner/mode", 100);
    arrivePub = nh.advertise<ftm_planning::Mission_arrive>("mission_alt_arrive", 100);
    localSub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 100, &local_planner::tfCallback, this);
    stateSub = nh.subscribe<mavros_msgs::State>("/mavros/state", 100, &local_planner::stateCallback, this);
    wpSub = nh.subscribe<geometry_msgs::PoseStamped>("/cord_transformer/wp_point", 100, &local_planner::wpcallback, this);
    mwpSub = nh.subscribe<geometry_msgs::PoseStamped>("/cord_transformer/mwp_point", 100, &local_planner::mwpcallback, this);
    YawSub = nh.subscribe<geometry_msgs::PoseStamped>("mission_planner/move_yaw", 100, &local_planner::s4yCallback, this);
    mission_resultSub = nh.subscribe<std_msgs::Bool>("mission_planner/mission_result", 100, &local_planner::mrCallback, this);
    mission_arrive.num = 1;
    mission_arrive.signal = false;

    gpsSub = nh.subscribe<sensor_msgs::NavSatFix>("/mavros/global_position/global", 100, &local_planner::gpsCallback, this);

  }
  ~local_planner()
  {
    cout << "Type : local_planner object deleted" << endl;
  }
  void fb1Callback(const geometry_msgs::PoseStamped::ConstPtr &msg){
    forbiddenCenter1.pose.position = msg->pose.position;
  }
  void fb2Callback(const geometry_msgs::PoseStamped::ConstPtr &msg){
    forbiddenCenter2.pose.position = msg->pose.position;
  }
  void tfCallback(const geometry_msgs::PoseStamped::ConstPtr &msg) //현재 로컬 좌표
  {
    lcl_drone_coordinate.pose = msg->pose;
    lcl_drone_coordinate.header = msg->header;
  }
  void wpcallback(const geometry_msgs::PoseStamped::ConstPtr &msg) //반환점 저장
  {
    waypoint.pose.position = msg->pose.position;
  }
  void mwpcallback(const geometry_msgs::PoseStamped::ConstPtr &msg) //미션포인트 저장
  {
    missionwayPoints.pose.position = msg->pose.position;
  }
  void s4yCallback(const geometry_msgs::PoseStamped::ConstPtr &msg) //yaw control
  {
    if (mode == MISSION)
    {
      next_waypoint.pose.orientation = msg->pose.orientation;
    }
  }
  void stateCallback(const mavros_msgs::State::ConstPtr &msg)
  {
    state = *msg;
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;
    if (mode == GROUND && msg->armed)
    {
      ROS_INFO("Vehicle armed");
      mode = TAKEOFF;
      ARM = true;
    }
    else
    {
      //ROS_ERROR("Arming failed");
    }
  }
  void mrCallback(const std_msgs::Bool::ConstPtr &msg)
  {
    mission_result = msg->data;
  }
  void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr &msg) //home 저장용
  {
    if (!_global_home_exist)
    {
      home_coordinate.latitude = msg->latitude;
      home_coordinate.longitude = msg->longitude;
      home_coordinate.altitude = msg->altitude;
      _global_home_exist=true;
    }
  }
  /*********************************************************************************/
  int ArriveCheck()
  {
    if ((abs(lcl_drone_coordinate.pose.position.x) < 0.5) &&
        (abs(lcl_drone_coordinate.pose.position.y) < 0.5))
      return 1; //AT HOME
    else if ((abs(lcl_drone_coordinate.pose.position.x - missionwayPoints.pose.position.x) < 0.1) &&
             (abs(lcl_drone_coordinate.pose.position.y - missionwayPoints.pose.position.y) < 0.1))
      return 2; //AT MISSION POINT
    else if ((abs(lcl_drone_coordinate.pose.position.x - waypoint.pose.position.x) < 0.2) &&
             (abs(lcl_drone_coordinate.pose.position.y - waypoint.pose.position.y) < 0.2))
      return 3; //AT WAYPOINT
    else
      return 0; //IS MOVING
  }
  void offboard()
  {
    switch (ArriveCheck())
    {
    case 0: // IS MOVING
      break;
    case 1: // AT HOME
      if (mission_arrive.num == 3){
        mode = LAND;}

      break;
    case 2: // AT MISSION POINT
      if (mission_arrive.num == 1 && !wparrive)
      {
        mode = MISSION;
        break;
      }
      else if (mission_arrive.num == 2 && !wparrive)
      {
        forbiddenCenter.pose = forbiddenCenter1.pose;
        r.data = radius1;
        next_waypoint.pose.position = waypoint.pose.position;
        break;
      }
      else if (mission_arrive.num == 2 && wparrive)
      {
        mode = MISSION;
        break;
      }
      else if (mission_arrive.num == 3 && wparrive)
      {
        forbiddenCenter.pose = forbiddenCenter1.pose;
        r.data = radius1;
        next_waypoint.pose.position.x = 0;
        next_waypoint.pose.position.y = 0;
        break;
      }
      else
        break;
    case 3: // AT WAYPOINT
      wparrive = true;
      forbiddenCenter.pose = forbiddenCenter1.pose;
      r.data = radius1;
      next_waypoint.pose.position = missionwayPoints.pose.position;
      break;
    }
  }
  void mission()
  {
    if (mission_arrive.num == 1 && !mission_result)
    {
      next_waypoint.pose.position.z = 3;
      if (lcl_drone_coordinate.pose.position.z <= 3.1)
      {
        mission_arrive.signal = true;
      }
    }
    else if (mission_arrive.num == 1 && mission_result)
    {
      if (lcl_drone_coordinate.pose.position.z < flight_level-0.1)
        next_waypoint.pose.position.z = flight_level;
      else{
        forbiddenCenter.pose = forbiddenCenter1.pose;
        r.data = radius1;
        next_waypoint.pose.position = waypoint.pose.position;
        mission_arrive.signal = false;
        mission_arrive.num = 2;
        mode = AVOIDANCE;
      }
    }
    else if (mission_arrive.num == 2 && !mission_result)
    {
      next_waypoint.pose.position.z = 3;
      if (lcl_drone_coordinate.pose.position.z <= 3.1)
        mission_arrive.signal = true;
    }
    else if (mission_arrive.num == 2 && mission_result)
    {
      if (lcl_drone_coordinate.pose.position.z < flight_level-0.1)
        next_waypoint.pose.position.z = flight_level;
      else{
        forbiddenCenter.pose = forbiddenCenter1.pose;
        r.data = radius1;
        next_waypoint.pose.position.x = 0;
        next_waypoint.pose.position.y = 0;
        mission_arrive.signal = false;
        mission_arrive.num = 3;
        mode = AVOIDANCE;
      }
    }

  }
  void takeoff()    //stabilized 모드이거나 pose 할 때 takeoff 안 하도록
  {
    if (lcl_drone_coordinate.pose.position.z > flight_level)
    {
      mode = AVOIDANCE;
      forbiddenCenter.pose = forbiddenCenter1.pose;
      r.data = radius1;
      next_waypoint.pose.position = missionwayPoints.pose.position;
    }
  }
  /*********************************************************************************/
  void takeOff() {
    mavros_msgs::CommandTOL takeoff_cmd;
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    switch(takeoff_stage) {
    case 0:
      takeoff_cmd.request.min_pitch = 0;
      takeoff_cmd.request.longitude = home_coordinate.longitude;
      takeoff_cmd.request.latitude = home_coordinate.latitude;
      takeoff_cmd.request.altitude = 100;
      takeoff_cmd.request.yaw = 0;
      take_off_client.call(takeoff_cmd);
      if(takeoff_cmd.response.success) {
        ROS_INFO_ONCE("[Planner] Requested takeoff.");
        takeoff_stage++;
      }else ROS_INFO_ONCE("[Planner] Fail request takeoff. Retrying...");
      break;
    case 1:

      if(lcl_drone_coordinate.pose.position.z > flight_level - 0.5) {
        geometry_msgs::PoseStamped pose_cmd;
        pose_cmd.pose.position.x = 0;
        pose_cmd.pose.position.y = 0;
        pose_cmd.pose.position.z = flight_level;
        target_pub.publish(pose_cmd);
        if( set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent && state.mode == "OFFBOARD"){
          ROS_INFO_ONCE("[Planner] Offboard returned.");
          takeoff_stage++;
          ROS_INFO_ONCE("[Planner] SUCCESS takeoff.");
          mode = AVOIDANCE;
          forbiddenCenter.pose = forbiddenCenter1.pose;
          r.data = radius1;
          next_waypoint.pose.position = missionwayPoints.pose.position;
        }else {
          ROS_INFO_ONCE("[Planner] Fail returning offboard. Retrying...");
        }


      }
      break;

    case 2:

//      pose_cmd.pose.position.x = 0;
//      pose_cmd.pose.position.y = 0;
//      pose_cmd.pose.position.z = 20;
//      pose_cmd.pose.orientation = setYaw(0, 0);
//      target_pub.publish(pose_cmd);

//      if(lcl_drone_coordinate.pose.position.z > flight_level - 0.5){
//        ROS_INFO_ONCE("[Planner] SUCCESS takeoff.");
//        mode = AVOIDANCE;
//        forbiddenCenter.pose = forbiddenCenter1.pose;
//        r.data = radius1;
//        next_waypoint.pose.position = missionwayPoints.pose.position;
//      }

      break;
    }


  }

  void land(){
    mavros_msgs::CommandTOL land_cmd;

      switch(land_stage) {

      case 1:
        ROS_INFO_ONCE("[Planner] Requesting land.");
        land_cmd.request.min_pitch = 0;
        land_cmd.request.yaw = 0;
        land_cmd.request.latitude = home_coordinate.latitude;
        land_cmd.request.longitude = home_coordinate.longitude;
        land_cmd.request.altitude = 0.0;
        land_client.call(land_cmd);
        if(land_cmd.response.success) {
          ROS_INFO("[Planner] SUCCESS land request.");
          land_stage++;
        }
        break;
      case 2:
        break;
      }

    }



  void change()
  {
    switch (mode)
    {
    case GROUND:
      break;
    case TAKEOFF:
      takeOff();
      break;
    case AVOIDANCE:
    case OFFBOARD:
      offboard();
      break;
    case LAND:
      land();
      break;
    case MISSION:
      mission();
      break;
    }
  }
  void localplannerPub()
  {
    mode_msg.data = mode;
    modePub.publish(mode_msg);
    fb_pub.publish(forbiddenCenter);
    arrivePub.publish(mission_arrive);
    next_wpPub.publish(next_waypoint);
    r_Pub.publish(r);
  }
  void core()
  {
    localplannerPub();
    change();
    //    localplannerPub();
  }
  void spin()
  {
    ros::Rate r(30);
    while (ros::ok())
    {
      core();
      r.sleep();
      ros::spinOnce();
    }
  }
};
int main(int argc, char **argv)
{
  ros::init(argc, argv, "local_planner");
  local_planner *_local_planner = new local_planner();
  _local_planner->spin();
  if (!ros::ok())
  {
    delete _local_planner;
  }
}
