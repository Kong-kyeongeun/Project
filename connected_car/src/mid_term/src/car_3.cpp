#include<actionlib/server/simple_action_server.h>
#include"mid_term/Action1Action.h"
#include"mid_term/Service1.h"
#include"mid_term/Topic1.h"
#include<ros/ros.h>
#include<iostream>
#include<cstdlib>
#include<ctime>
#include<math.h>

#define _driving 0
#define _waitting 1
#define _accident 2

using namespace std;
int home_x;
int home_y;

class _car_3{
    protected:
    ros::NodeHandle nh;

    ros::Publisher car_3_pub = nh.advertise<mid_term::Topic1>("car_3/information",1000);

    ros::ServiceServer _brake = nh.advertiseService("control_center/control_speed", &_car_3::brake, this);

    mid_term::Topic1 msg1;
    mid_term::Service1 srv;

    int mode = _driving;
    double x, y;
    double speed;

    public:
    _car_3(){
      x = 0.0;
      y = 0.0;
      speed = 0.1;

  }

    ~_car_3(void) {}

    void control_speed(){
      if(mode == _driving){
        if(speed <= 0.1 || rand()%100 > 20)
          speed +=0.1;
        else
          speed -= 0.1;
      }
  }

    void change_loc(){
      if(mode == _driving){
        x += speed;
        y += 2*speed;
      }
      ROS_INFO("car_3 speed : %f" ,speed*10);
  }
    void arrive_home(){
      if(x>=home_x&&y >=home_y)
        ros::shutdown();
    }

    void pub_car_3(){
      msg1.car_num = 3;
      msg1.x = x;
      msg1.y = y;
      msg1.speed = speed;
      car_3_pub.publish(msg1);
    }

    bool brake(mid_term::Service1::Request &req, mid_term::Service1::Response &res)
    {
      if(req.a == true)
      {
        speed = 0.3;
        res.b = 3;
      }
      else
        res.b = speed*10;
      ROS_INFO("brake!");
      return true;
    }


    void spin(){
      ros::Rate r(1);
      while(ros::ok()){
        nh.param("car_3/home_x", home_x, 50);
        nh.param("car_3/home_y", home_y, 100);
        control_speed();
        change_loc();
        pub_car_3();
        arrive_home();
        r.sleep();
        ros::spinOnce();
      }
    }
};

int main(int argc , char** argv){
  srand((unsigned int)time(NULL));
  ros::init(argc,argv,"car_3");
  _car_3 * car = new _car_3();
  car->spin();
  if(!ros::ok()){
    delete car;
  }
}
