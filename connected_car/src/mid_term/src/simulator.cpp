#include <ros/ros.h>
#include <std_msgs/Int8.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include"mid_term/Topic1.h"
#include"mid_term/Topic2.h"

using namespace std;

class simulator{
private:
  visualization_msgs::Marker accident_marker;
  visualization_msgs::Marker wp_marker;
  visualization_msgs::Marker wp_marker2;
  visualization_msgs::MarkerArray wp_marker_array;
  ros::NodeHandle n;
  ros::Subscriber accident_marker_sub;
  ros::Subscriber car_1_sub;
  ros::Subscriber car_2_sub;
  ros::Subscriber car_3_sub;
  ros::Subscriber police_sub;
  ros::Subscriber accident_loc_sub;
  // ros::Subscriber setpoint_sub_array;
  ros::Publisher accident_marker_pub;
  ros::Publisher wp_marker_pub;
  bool accidentrcv = false;

public:
       simulator(){
         //accident_marker_sub = n.subscribe<geometry_msgs::PoseStamped>("/accident_point",100,&simulator::accidentCallback,this);
         car_1_sub = n.subscribe("car_1/information",100,&simulator::wpCallback,this);
         car_2_sub= n.subscribe("car_2/information",100,&simulator::wpCallback,this);
         car_3_sub= n.subscribe("car_3/information",100,&simulator::wpCallback,this);
         police_sub= n.subscribe("police_car/information",100,&simulator::wpCallback2,this);
         accident_loc_sub= n.subscribe("control_center/accident_information",100,&simulator::accidentCallback,this);
         // setpoint_sub_array = n,subscribe<geometry_msgs::PoseStamped>('')
         accident_marker_pub = n.advertise<visualization_msgs::Marker>( "visualization_marker/accident", 0 );
         wp_marker_pub = n.advertise<visualization_msgs::MarkerArray>( "visualization_marker/wp", 0 );

         wp_marker.scale.x = 0.5;
         wp_marker.scale.y = 0.5;
         wp_marker.scale.z = 1.0;

         wp_marker.color.r = 0.0f;
         wp_marker.color.g = 1.0f;
         wp_marker.color.b = 0.0f;
         wp_marker.color.a = 0.5;

         wp_marker.lifetime = ros::Duration();

         accident_marker.scale.x = 5.0;
         accident_marker.scale.y = 5.0;
         accident_marker.scale.z = 100.0;

         accident_marker.color.r = 1.0f;
         accident_marker.color.g = 0.0f;
         accident_marker.color.b = 0.0f;
         accident_marker.color.a = 0.5;

         accident_marker.lifetime = ros::Duration(1);

         accident_marker.header.stamp = ros::Time::now();
         accident_marker.type = visualization_msgs::Marker::CYLINDER;
         accident_marker.ns = "basic_shapes";
         accident_marker.id = 0;
         accident_marker.header.frame_id = "world";
         accident_marker.action = visualization_msgs::Marker::ADD;

         wp_marker.header.stamp = ros::Time::now();
         wp_marker.type = visualization_msgs::Marker::SPHERE;
         wp_marker.ns = "basic_shapes";
         wp_marker.id = 1;
         wp_marker.header.frame_id = "world";
         wp_marker.action = visualization_msgs::Marker::ADD;
       }

       void wpCallback(const mid_term::Topic1ConstPtr&  msg){
          wp_marker.color.r = 0.0f;
          wp_marker.color.g = 1.0f;
          wp_marker.color.b = 0.0f;
          wp_marker.color.a = 0.5;
          wp_marker.pose.position.x = msg->x;
          wp_marker.pose.position.y = msg->y;
          wp_marker.pose.position.z = 0.0;
          wp_marker.pose.orientation.x = 0.0;
          wp_marker.pose.orientation.y = 0.0;
          wp_marker.pose.orientation.z = 0.0;
          wp_marker.pose.orientation.w = 1.0;
          wp_marker.id = msg->car_num;
          wp_marker_array.markers.push_back(wp_marker);
  }
       void wpCallback2(const mid_term::Topic1ConstPtr&  msg){
          wp_marker.color.r = 0.0f;
          wp_marker.color.g = 0.0f;
          wp_marker.color.b = 1.0f;
          wp_marker.color.a = 0.5;
          wp_marker.pose.position.x = msg->x;
          wp_marker.pose.position.y = msg->y;
          wp_marker.pose.position.z = 0.0;
          wp_marker.pose.orientation.x = 0.0;
          wp_marker.pose.orientation.y = 0.0;
          wp_marker.pose.orientation.z = 0.0;
          wp_marker.pose.orientation.w = 1.0;
          wp_marker.id = msg->car_num;
          wp_marker_array.markers.push_back(wp_marker);

  }

        void accidentCallback(const mid_term::Topic2ConstPtr&  msg){
          if(msg->accident)
          {
            accidentrcv= true;
            accident_marker.pose.position.x = msg->x;
            accident_marker.pose.position.y = msg->y;
            accident_marker.pose.position.z = 0;
            wp_marker.pose.orientation.x = 0.0;
            wp_marker.pose.orientation.y = 0.0;
            wp_marker.pose.orientation.z = 0.0;
            wp_marker.pose.orientation.w = 1.0;
          }
          else
            accidentrcv = false;
      }


       void publish(){
         wp_marker_pub.publish(wp_marker_array);
         if(accidentrcv==true)
           accident_marker_pub.publish(accident_marker);
      }
       void spin(){
         ros::Rate r(1);
           while(ros::ok()){
               publish();
               //core();
               r.sleep();
               ros::spinOnce();
           }
       }

};

int main(int argc, char** argv){
    ros::init(argc,argv,"simulator");
    simulator *_simulator = new simulator();

    if(!ros::ok()){
        delete _simulator;
    }
    _simulator->spin();
    return 0;
}
