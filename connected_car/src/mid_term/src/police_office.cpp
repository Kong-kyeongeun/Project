#include<actionlib/server/simple_action_server.h>
#include "mid_term/Action1Action.h"
#include "mid_term/Service1.h"
#include "mid_term/Service2.h"
#include "mid_term/Topic1.h"
#include "ros/ros.h"


class police_office{
    protected:
    ros::NodeHandle nh;
    ros::Publisher police_car_pub = nh.advertise<mid_term::Topic1>("police_car/information",1000);
    actionlib::SimpleActionServer<mid_term::Action1Action> action_server;
    std::string action_name_;
    mid_term::Action1Feedback feedback;
    mid_term::Action1Result result;
    mid_term::Service1 srv;
    mid_term::Topic1 msg1;
    double speed;
    double x,y;
    public:
    police_office(std::string name)
        : action_server(nh, name, boost::bind(&police_office::executeCB, this, _1),
            false),
            action_name_(name) {
        speed = 0.3;
        x = 0.0;
        y = 0.0;
        action_server.start();
      }

    ~police_office(void) {}

    void pub_police_car(){
      msg1.car_num = 4;
      msg1.x = x;
      msg1.y = y;
      msg1.speed = speed;
      police_car_pub.publish(msg1);
    }

    void executeCB(const mid_term::Action1GoalConstPtr& goal){
      ros::Rate r(1);
      while(!action_server.isPreemptRequested()|| ros::ok())
      {
        if((x >= goal->x && y >= goal->y) || sqrt(pow((goal->x)-x,2) +pow((goal->y)-y,2)) <= 0.1)
            {
                x = goal->x;
                y = goal->y;
                ROS_INFO("arrive at the accident site");
                ROS_INFO("current position_x : %f", x);
                ROS_INFO("current position_y : %f", y);
                result.x = x;
                result.y = y;
                action_server.setSucceeded(result);
                x = 0;
                y = 0;
                break;
            }
        else
        {
            ROS_INFO("moving to the accident site");
            ROS_INFO("current position_x : %f", x);
            ROS_INFO("current position_y : %f", y);
            x += 2*speed;
            y += speed;
            feedback.x = x;
            feedback.y = y;
            action_server.publishFeedback(feedback);
        }
        pub_police_car();
        r.sleep();
        ros::spinOnce();
      }

  }
};
    int main(int argc , char** argv){
        ros::init(argc, argv, "police_office");
        police_office po ("police_office");
        ros::Rate r(1);
        while(ros::ok())
        {
          po.pub_police_car();
          r.sleep();
          ros::spinOnce();
        }
        return 0;
    }


