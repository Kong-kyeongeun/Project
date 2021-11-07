include <iostream>
#include <ros/ros.h>
#include <std_msgs/Int8.h> // 도착 시그널 받을때 1, 2로 하면 필요할듯
#include <sensor_msgs/NavSatFix.h> //for current local
#include <std_msgs/Float64.h> //for compass_hdg
#include <sensor_msgs/MagneticField.h> //for compass data 안쓸듯 아마
#include <geometry_msgs/PoseStamped.h> // for yaw control
#include <tf/transform_broadcaster.h> //안쓸듯
#include <std_msgs/Bool.h> // for arduino commu
#include <std_msgs/Int16.h>
#include "ftm_planning/Mission_arrive.h" // for localplanner commu

#define waitting 0
#define yaw_control 1
#define first_micro_ing 2
#define bluetooth_ing 3
#define second_micro_ing 4
#define mission_complete 5


using namespace std;

class mission_planner{

private:

  std_msgs::Float64 current_hdg; //current_hdg
  std_msgs::Int16 micro_mission_request;
  std_msgs::Int16 bluetooth_mission_request;
  std_msgs::Bool mission_result;
  geometry_msgs::PoseStamped local_pose;
  geometry_msgs::PoseStamped Yaw;
//  geometry_msgs::PoseStamped waypoint; //이륙 후 임의의 첫 번째 waypoint (생성자에서 설정해도 무방)

  ros::NodeHandle nh;
//  ros::Publisher mission2_resultPub;
  ros::Publisher YawPub;
  ros::Publisher mission_resultPub;
  ros::Publisher micro_requestPub;
  ros::Publisher bluetooth_requestPub;

  //  ros::Subscriber
  ros::Subscriber hdgSub;
  ros::Subscriber localSub;
  ros::Subscriber arriveSub;
  ros::Subscriber micro_mission_resultSub;
  ros::Subscriber bluetooth_resultSub;

  double yawToMove = 0;
  double currentHdg = 0;
  int arrive_num = 0;
  int yaw_num = 0;
  int mode = waitting;
  tf::Quaternion q;

public:
  mission_planner(){
    YawPub = nh.advertise<geometry_msgs::PoseStamped>
        ("mission_planner/move_yaw",100);
    mission_resultPub = nh.advertise<std_msgs::Bool>
        ("mission_planner/mission_result",100);

    micro_requestPub = nh.advertise<std_msgs::Int16>
        ("mission_planner/micro_mission_request",100);

    bluetooth_requestPub = nh.advertise<std_msgs::Int16>
        ("mission_planner/bluetooth_mission_request",100);


    hdgSub = nh.subscribe<std_msgs::Float64>("/mavros/global_position/compass_hdg",100,&mission_planner::hdgCallback,this); //hdg
    localSub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose",100,&mission_planner::localCallback,this);
    arriveSub = nh.subscribe<ftm_planning::Mission_arrive>("mission_alt_arrive",100,&mission_planner::arriveCallback,this);
    micro_mission_resultSub = nh.subscribe<std_msgs::Int16>("micro/micro_mission_result",100,&mission_planner::micro_resultCallback,this);//dsdsd
    bluetooth_resultSub = nh.subscribe<std_msgs::Int16>("bluetooth/bluetooth_mission_result",100,&mission_planner::bluetooth_resultCallback,this);
  }

  ~mission_planner(){
    cout << "Type : mission_planner object deleted" << endl;
  }

// callback 함수
  void hdgCallback(const std_msgs::Float64::ConstPtr& msg){
    //    current_hdg.data=msg->data;
    currentHdg=msg->data;
  }

  void arriveCallback(const ftm_planning::Mission_arrive::ConstPtr& msg) // msg 수정필요
  {
    if(msg->signal&&(mode == waitting)){ // 1 0
      mode = yaw_control;
      arrive_num = msg->num;
    }
    else if(!(msg->signal)&&(mode == mission_complete)){ // 0 1
      //파라미터 초기화
      mode = waitting;
      yaw_num = 0;
    }
    // else 생각
  }

  void micro_resultCallback(const std_msgs::Int16::ConstPtr& msg){
    if(mode == first_micro_ing|| mode == second_micro_ing)
    {
      if(msg->data == 1)
        mode = bluetooth_ing;
      else if(msg->data == 2)
        mode = mission_complete;
    }
  }

  void bluetooth_resultCallback(const std_msgs::Int16::ConstPtr& msg){
    if(mode == bluetooth_ing)
    {
      if(arrive_num == msg->data)
        mode = second_micro_ing;
    }
      }

  void localCallback(const geometry_msgs::PoseStamped::ConstPtr& msg){ //yaw 트
    if(!(mode == waitting)){
      cout<<"setQ"<<endl;
      q.setX(msg->pose.orientation.x);
      q.setY(msg->pose.orientation.y);
      q.setZ(msg->pose.orientation.z);
      q.setW(msg->pose.orientation.w);
      tf::Matrix3x3 RPY(q);
      double R,P,Y;
      RPY.getRPY(R,P,Y);
      Y= Y + 0.01745329*yawToMove;//Y+yawToMove; //Radian
      q.setX(sin(R/2)*cos(P/2)*cos(Y/2)-cos(R/2)*sin(P/2)*sin(Y/2));
      q.setY(cos(R/2)*sin(P/2)*cos(Y/2)+sin(R/2)*cos(P/2)*sin(Y/2));
      q.setZ(cos(R/2)*cos(P/2)*sin(Y/2)-sin(R/2)*sin(P/2)*cos(Y/2));
      q.setW(cos(R/2)*cos(P/2)*cos(Y/2)+sin(R/2)*sin(P/2)*sin(Y/2));
      Yaw.pose.orientation.x=q.getX();
      Yaw.pose.orientation.y=q.getY();
      Yaw.pose.orientation.z=q.getZ();
      Yaw.pose.orientation.w=q.getW();
      YawPub.publish(Yaw); // !! 플래너 노드에서 setpoint를 계속주고있으면 안됨 orientation 바뀌어버림 , !!yaw 통제 할때 position도 통제해 줘야함 드론이 흐름
    }
  }

  // yaw 계산 함수
  void get_yawToMove(){
    if(!(mode == waitting)&&arrive_num == 1){ //first mission
      if(currentHdg>90.0&&currentHdg<270.0) yawToMove = -(180.0-currentHdg);
      else if (currentHdg>=270.0) yawToMove = -(360.0-currentHdg);
      else if (currentHdg<=90.0) yawToMove = currentHdg;
     }
    else if (!(mode == waitting)&&arrive_num == 2) { //second mission
      if(currentHdg>=0.0&&currentHdg<=180.0) yawToMove = -(90.0-currentHdg);
      else if (currentHdg<=360.0&&currentHdg>=180.0) yawToMove = -(270.0-currentHdg);
     }
  }

 void set_yaw(){
   if(mode == yaw_control)
   {
     if(yaw_num < 10){
       if(abs(yawToMove) < 3)
           yaw_num++;
       else
         yaw_num = 0;
     }
     else if(yaw_num >= 10){
       mode = first_micro_ing;
       cout << mode << endl;
     }
   }
 }

  //publish 함수
  void mission_request2micro() //ard 어떻게 받는지 몰라서 나눠놓긴함
  {
    if(mode == first_micro_ing)
      micro_mission_request.data = 1;

    else if(mode == second_micro_ing)
      micro_mission_request.data = 2;

    else
      micro_mission_request.data = 0;
  micro_requestPub.publish(micro_mission_request);
 }

  void mission_reqeust2bluetooth()
  {
   // issue
   if(mode == bluetooth_ing)
      bluetooth_mission_request.data = arrive_num;

   bluetooth_requestPub.publish(bluetooth_mission_request);
  }



  void mission_result2planner()
  {
    if(mode == mission_complete)
      mission_result.data = true;
    else
      mission_result.data = false;

    mission_resultPub.publish(mission_result);
  }



  void spin(){
    ros::Rate r(30);
    while(ros::ok()){
      get_yawToMove();
      set_yaw();
      mission_request2micro();
      mission_reqeust2bluetooth();
      mission_result2planner();
      cout << "mode : " << mode  << endl;
      r.sleep();
      ros::spinOnce();
    }
  }
};

int main(int argc, char** argv){
  ros::init(argc,argv,"mission_planner");
  mission_planner *_mission_planner = new mission_planner();
  _mission_planner->spin();
  if(!ros::ok()){
    delete _mission_planner;
  }
}
