#include <iostream>
#include <ros/ros.h> //ros
#include <std_msgs/Int8.h>
#include <sensor_msgs/NavSatFix.h>
#include <string>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandBool.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

class avoidance_simulator{


private:
    ros::NodeHandle nh;

    ros::Subscriber wp_sub;
    ros::Subscriber fb_sub;
    ros::Subscriber localSub;
    ros::Subscriber setpoint_sub;

//    ros::Publisher wp_pub;
//    ros::Publisher fb_pub;
    ros::Publisher markerArrayPub;
    ros::Publisher lclMarkerPub;
    ros::Publisher LineArrayPub;
    visualization_msgs::MarkerArray setpointMarkerArray;
    visualization_msgs::MarkerArray D_WPArray;

    geometry_msgs::PoseStamped waypoint;

    bool getWP = false;

public:
    avoidance_simulator(){
    wp_sub = nh.subscribe<geometry_msgs::PoseStamped>("/wp_point",100,&avoidance_simulator::wpCallback,this);
    fb_sub = nh.subscribe<geometry_msgs::PoseStamped>("/forb_point",100,&avoidance_simulator::fbCallback,this);
    localSub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose",100,&avoidance_simulator::localCallback,this);

    markerArrayPub = nh.advertise<visualization_msgs::MarkerArray>("/viz_marker",1000);
    LineArrayPub = nh.advertise<visualization_msgs::MarkerArray>("/Line",1000);
    lclMarkerPub = nh.advertise<visualization_msgs::Marker>("/localPosition",1000);
    }
    ~avoidance_simulator(){

    }
    visualization_msgs::Marker markergenerator(geometry_msgs::PoseStamped point,double scale,double scale_z){
        visualization_msgs::Marker marker;
        marker.header.stamp = ros::Time::now();
        marker.type = visualization_msgs::Marker::CYLINDER;
        marker.ns = "basic_shapes";
        marker.id = int(ros::Time::now().nsec);
        marker.pose.position.x = point.pose.position.x; //임의의 비행금지구역 좌표
        marker.pose.position.y = point.pose.position.y; //임의의 비행금지구역 좌표
        marker.pose.position.z = point.pose.position.z;
        marker.pose.orientation.x = 0;
        marker.pose.orientation.y = 0;
        marker.pose.orientation.z = 0;
        marker.pose.orientation.w = 1.0;

        marker.scale.x = scale; //임의의 비행금지구역 반경 (x,y 같게)
        marker.scale.y = scale;
        marker.scale.z = scale_z; //비행금지구역 높이는 무제한

        marker.color.r = 1.0f;
        marker.color.g = scale;
        marker.color.b = 1.0f;
        marker.color.a = 0.5; //마커 색상 및 투명도

        marker.lifetime = ros::Duration(); //마커 지속시간
        marker.header.frame_id = "map";
        return marker;
    }
    visualization_msgs::Marker markergenerator(geometry_msgs::PoseStamped point,double scale,double scale_z, int _id){
        visualization_msgs::Marker marker;
        marker.header.stamp = ros::Time::now();
        marker.type = visualization_msgs::Marker::CYLINDER;
        marker.ns = "basic_shapes";
        marker.id = _id;
        marker.pose.position.x = point.pose.position.x;
        marker.pose.position.y = point.pose.position.y;
        marker.pose.position.z = point.pose.position.z;
        marker.pose.orientation.x = 0;
        marker.pose.orientation.y = 0;
        marker.pose.orientation.z = 0;
        marker.pose.orientation.w = 1.0;

        marker.scale.x = scale; //임의의 비행금지구역 반경 (x,y 같게)
        marker.scale.y = scale;
        marker.scale.z = scale_z; //비행금지구역 높이는 무제한

        marker.color.r = 1.0f;
        marker.color.g = 1;
        marker.color.b = 0.0f;
        marker.color.a = 0.5; //마커 색상 및 투명도

        marker.lifetime = ros::Duration(); //마커 지속시간
        marker.header.frame_id = "map";
        return marker;
    }
    visualization_msgs::Marker markergenerator(double x, double y,int _id){
        visualization_msgs::Marker marker;
        marker.header.stamp = ros::Time::now();
        marker.type = visualization_msgs::Marker::CYLINDER;
        marker.ns = "basic_shapes";
        marker.id = _id;
        marker.pose.position.x = x;
        marker.pose.position.y = y;
        marker.pose.position.z = 10;
        marker.pose.orientation.x = 0;
        marker.pose.orientation.y = 0;
        marker.pose.orientation.z = 0;
        marker.pose.orientation.w = 1.0;

        marker.scale.x = 0.2;
        marker.scale.y = 0.2;
        marker.scale.z = 0.2;

        marker.color.r = 0.0f;
        marker.color.g = 0.0f;
        marker.color.b = 1.0f;
        marker.color.a = 0.5; //마커 색상 및 투명도

        marker.lifetime = ros::Duration(); //마커 지속시간
        marker.header.frame_id = "map";
        return marker;
    }
    void wpCallback(const geometry_msgs::PoseStamped::ConstPtr& msg){
        geometry_msgs::PoseStamped wp;
        waypoint.header = msg->header;
        waypoint.pose = msg->pose;
        wp.pose = msg->pose;
        D_WPArray.markers.push_back(markergenerator(wp,0.5,0.5));
        getWP = true;
    }
    void fbCallback(const geometry_msgs::PoseStamped::ConstPtr& msg){
        geometry_msgs::PoseStamped fb;
        fb.pose = msg->pose;
        D_WPArray.markers.push_back(markergenerator(fb,48,50,404));
    }
    void localCallback(const geometry_msgs::PoseStamped::ConstPtr& msg){
        visualization_msgs::MarkerArray LineArray;
        geometry_msgs::PoseStamped Dp;
        Dp.pose = msg->pose;
        lclMarkerPub.publish(markergenerator(Dp,0.3,0.3));
        if(getWP&&(abs((Dp.pose.position.x-waypoint.pose.position.x))>0.1)){
            double m = (Dp.pose.position.y-waypoint.pose.position.y)/(Dp.pose.position.x-waypoint.pose.position.x);
            for(int x = -200; x<200; x++){
                double y = m*(0.5*x-Dp.pose.position.x)+Dp.pose.position.y;
                LineArray.markers.push_back(markergenerator(0.5*x,y,x));
            }
            LineArrayPub.publish(LineArray);
        }
    }
    void core(){
    markerArrayPub.publish(D_WPArray);
    }
    void spin(){
      ros::Rate r(30);
        while(ros::ok()){
             core();
            r.sleep();
            ros::spinOnce();
        }
    }
};

int main(int argc, char** argv){
    ros::init(argc,argv,"avoidance_simulator");
    //OOP + dynamic allocation adapted.
    avoidance_simulator *_avoidance_simulator = new avoidance_simulator();
    _avoidance_simulator->spin();
    if(!ros::ok()){
        delete _avoidance_simulator;
    }
}
