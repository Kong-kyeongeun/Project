#include "actionlib/client/simple_action_client.h"
#include "actionlib/client/terminal_state.h"
#include"mid_term/Action1Action.h"
#include"mid_term/Service1.h"
#include"mid_term/Service2.h"
#include"mid_term/Topic1.h"
#include"mid_term/Topic2.h"
#include<ros/ros.h>
#include<iostream>
#include<cstdlib>
#include<ctime>
#include<math.h>

#define _driving 0
#define _waitting 1
#define _accident 2

using namespace std;
class _car_1{
    protected:
    ros::NodeHandle nh;
    ros::Rate r;
    ros::Publisher car_1_pub = nh.advertise<mid_term::Topic1>("car_1/information",1000);
    ros::Subscriber car_1_sub = nh.subscribe("control_center/accident_information", 1000, &_car_1::car_1Callback, this);

    ros::ServiceClient occur_accident = nh.serviceClient<mid_term::Service2>("car_1/accident_information");

    actionlib::SimpleActionClient<mid_term::Action1Action> action_client;

    mid_term::Topic1 msg1;
    mid_term::Service2 srv;
    mid_term::Action1Feedback feedback;
    mid_term::Action1Result result;

    int mode = _driving;
    double x, y;
    double speed;

    public:
    _car_1(): r(1), action_client("police_office", true){
      x = 0.0;
      y = 0.0;
      speed = 0.2;
  }

    ~_car_1(void) {}

    void change_loc(){
      if(mode == _driving){
        x += 2*speed;
        y += speed;
      }
  }

    void accident(){
      if(mode == _driving){
        if(rand()%100 > 90){ // 10 percent
          mode = _accident;
          srv.request.x = x;
          srv.request.y = y;
          srv.request.accident = true;
          ROS_INFO("Occur accident!");
          occur_accident.call(srv);
          speed = 0;
          ROS_INFO("count of nearby car : %d", srv.response.b);
          }
      }
  }


    void call_police(){
      if(mode == _accident)
      {
        ROS_INFO("Waiting for action server to start");
        action_client.waitForServer();

        ROS_INFO("Action server started, sending goal.");
        mid_term::Action1Goal goal;
        goal.x = x;
        goal.y = y;
        action_client.sendGoal(goal);
        while(!(action_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED))
        {
          ROS_INFO("police comming");
          pub_car_1();
          r.sleep();
          ros::spinOnce();
        }
        ROS_INFO("arrive police!");
        mode = _driving;
        speed = 0.2;
        srv.request.accident = false;
        occur_accident.call(srv);
      }
    }

    void pub_car_1(){
      msg1.car_num = 1;
      msg1.x = x;
      msg1.y = y;
      msg1.speed = speed;
      car_1_pub.publish(msg1);
    }


    void car_1Callback(const mid_term::Topic2::ConstPtr& msg)
    {
      if(msg->accident)
      {
        ROS_INFO("accident_inforation!");
        ROS_INFO("accident_x : %f", msg->x);
        ROS_INFO("accident_y : %f", msg->y);
      }
    }

    void spin(){
      while(ros::ok()){

        change_loc();
        accident();
        call_police();
        pub_car_1();
        r.sleep();
        ros::spinOnce();
      }
    }
};

int main(int argc , char** argv){
  srand((unsigned int)time(NULL));
  ros::init(argc,argv,"car_1");
  _car_1 * car = new _car_1();
  car->spin();
  if(!ros::ok()){
    delete car;
  }
}
