#include"mid_term/Service1.h"
#include"mid_term/Service2.h"
#include"mid_term/Topic1.h"
#include"mid_term/Topic2.h"
#include<ros/ros.h>



class control_center{
    protected:
    ros::NodeHandle nh;

    ros::Publisher accident_loc_pub = nh.advertise<mid_term::Topic2>("control_center/accident_information",1000);

    ros::Subscriber car_1_sub = nh.subscribe("car_1/information", 1000, &control_center::car_1Callback, this);
    ros::Subscriber car_2_sub = nh.subscribe("car_2/information", 1000, &control_center::car_2Callback, this);
    ros::Subscriber car_3_sub = nh.subscribe("car_3/information", 1000, &control_center::car_3Callback, this);

    ros::ServiceServer take_accident = nh.advertiseService("car_1/accident_information", &control_center::happening, this );
    ros::ServiceClient control_speed = nh.serviceClient<mid_term::Service1>("control_center/control_speed");

    mid_term::Topic2 msg2;
    mid_term::Service1 srv;
    double accident_x, accident_y;
    bool _accident = false;
    bool finished_before_timeout;

    public:
    control_center()
    {
      accident_x = 0;
      accident_y = 0;
      _accident = false;
  }

    ~control_center(void) {}

    void car_1Callback(const mid_term::Topic1::ConstPtr& msg)
    {
      ROS_INFO("----INFORMATION CAR_1----");
      ROS_INFO("car_1_x : %f", msg->x);
      ROS_INFO("car_1_y : %f", msg->y);
      ROS_INFO("car_1_speed : %f", msg->speed);
    }

    void car_2Callback(const mid_term::Topic1::ConstPtr& msg)
    {
      ROS_INFO("----INFORMATION CAR_2----");
      ROS_INFO("car_2_x : %f", msg->x);
      ROS_INFO("car_2_y : %f", msg->y);
      ROS_INFO("car_2_speed : %f", msg->speed);
    }

    void car_3Callback(const mid_term::Topic1::ConstPtr& msg)
    {
      ROS_INFO("----INFORMATION CAR_3----");
      ROS_INFO("car_3_x : %f", msg->x);
      ROS_INFO("car_3_y : %f", msg->y);
      ROS_INFO("car_3_speed : %f", msg->speed);

      if(msg->speed > 1)
      {
        srv.request.a = true;
        control_speed.call(srv);
        ROS_INFO("car_3 changing speed : %d", srv.response.b);
      }
    }

    void pub_accident_loc(){
      if(_accident)
      {
        msg2.x = accident_x;
        msg2.y = accident_y;
        msg2.accident = true;
        accident_loc_pub.publish(msg2);
      }
      else {
        msg2.x = 0.0;
        msg2.y = 0.0;
        msg2.accident = false;
        accident_loc_pub.publish(msg2);

      }
    }

    bool happening(mid_term::Service2::Request &req, mid_term::Service2::Response &res) //warning
    {
      if(req.accident == true)
      {
        accident_x = req.x;
        accident_y = req.y;
        _accident = true;
        res.b = 1;

      }
      else{
        _accident = false;
        res.b = 0;
      }
      return true;
  }


    void spin(){
      ros::Rate r(1);
      while(ros::ok()){
        pub_accident_loc();
        r.sleep();
        ros::spinOnce();
      }
    }
};

int main(int argc , char** argv){
  ros::init(argc,argv,"control_center");
  control_center * cc = new control_center();
  cc->spin();
  if(!ros::ok()){
    delete cc;
  }
}

/*
void check119()
{
  if(_accident)
  {
    ROS_INFO("Waiting for action server to start");
    action_client.waitForServer();

    ROS_INFO("Action server started, sending goal.");
    mid_term::Action1Goal goal;
    goal.x = accident_x;
    goal.y = accident_y;


    ROS_INFO("checking");
    if(client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)  // warning
      ROS_INFO("clear");
      _accident = false;

  }
}
*/
