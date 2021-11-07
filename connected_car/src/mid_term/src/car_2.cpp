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

using namespace std;

class _car_2{
    protected:
    ros::NodeHandle nh;

    ros::Publisher car_2_pub = nh.advertise<mid_term::Topic1>("car_2/information",1000);
    ros::Subscriber car_2_sub = nh.subscribe("control_center/accident_information", 1000, &_car_2::car_2Callback, this);

    mid_term::Topic1 msg1;
    mid_term::Service2 srv;


    int mode = _driving;
    double x, y;
    double speed;

    public:
    _car_2(){
      x = 0.0;
      y = 0.0;
      speed = 0.1;

  }

    ~_car_2(void) {}

    void change_loc(){
      if(mode == _driving){
        x += 2*speed;
        y += speed;
      }
  }

    void pub_car_2(){
      msg1.car_num = 2;
      msg1.x = x;
      msg1.y = y;
      msg1.speed = speed;
      car_2_pub.publish(msg1);
    }

    void car_2Callback(const mid_term::Topic2::ConstPtr& msg)
    {
      if(msg->accident)
      {
        if(x < msg->x && y < msg->y && sqrt(pow((msg->x)-x,2) +pow((msg->y)-y,2)) < 2.25 )
        {
         ROS_INFO("An accident occurred nearby");
         ROS_INFO("waitting");
          mode = _waitting;
          speed = 0.0;
        }
      }
      else {
        ROS_INFO("running");
        mode = _driving;
        speed = 0.1;
      }
    }

    void spin(){
      ros::Rate r(1);
      while(ros::ok()){
        change_loc();
        pub_car_2();
        r.sleep();
        ros::spinOnce();
      }
    }
};

int main(int argc , char** argv){  
  srand((unsigned int)time(NULL));
  ros::init(argc,argv,"car_2");
  _car_2 * car = new _car_2();
  car->spin();
  if(!ros::ok()){
    delete car;
  }
}
