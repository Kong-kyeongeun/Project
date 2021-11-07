#include <iostream>
#include <ros/ros.h> //ros
#include <std_msgs/Int8.h>
#include <sensor_msgs/NavSatFix.h>
#include <string>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandBool.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>
#include <cmath>
#define GROUND 0
#define TAKEOFF 1
#define OFFBOARD 2
#define LAND 3
#define MISSION 4
#define AVOIDANCE 5

class avoidance_operator{

private:
    int mode = GROUND;
//    int r1;//
//    int r2;//
    int r;// = nh.param("/avoidance_operator/radius",24); //forbidden part radius
    int s = 0; //safety factor
    double e = 1; //gnss error
    double theta; //= 2*asin(e/(s+r));
    geometry_msgs::PoseStamped waypoint;
    geometry_msgs::PoseStamped dronePose;
    geometry_msgs::PoseStamped forbiddenCenter;
    geometry_msgs::PoseStamped N;
    double X;
    double Y;
    double p;
    double q;
    double k; // DN = k*DWp
    bool getWp = false;
    bool getFb = false;

    ros::NodeHandle nh;

    ros::Subscriber wp_sub;
    ros::Subscriber fb_sub;
    ros::Subscriber localSub;
    ros::Subscriber modeSub;
    ros::Subscriber r_Sub;
    ros::Publisher N_pub;
    ros::Publisher S_pub;
    ros::Publisher G_pub;
    ros::Publisher setpoint_pub;

public:
    avoidance_operator(){
//    int r1 = nh.param("/avoidance_operator/radius1",24);
//    int r2 = nh.param("/avoidance_operator/radius2",24);
    N_pub = nh.advertise<geometry_msgs::PoseStamped>("/n_point",100);
    S_pub = nh.advertise<geometry_msgs::PoseStamped>("/s_point",100);
    G_pub = nh.advertise<geometry_msgs::PoseStamped>("/g_point",100);
    localSub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose",100,&avoidance_operator::dronePoseCallback,this);
    wp_sub = nh.subscribe<geometry_msgs::PoseStamped>("/diagnose/next_waypoint",100,&avoidance_operator::wpCallback,this);
    fb_sub = nh.subscribe<geometry_msgs::PoseStamped>("/local_planner/forb_point",100,&avoidance_operator::fbCallback,this);
    modeSub = nh.subscribe("/local_planner/mode", 100, &avoidance_operator::modeCallback, this);
    r_Sub = nh.subscribe("/local_planner/radius", 100, &avoidance_operator::radiusCallback, this);
    }
    void radiusCallback(const std_msgs::Int8 &msg){
        r = msg.data;
        theta = 2*asin(e/(s+r));
    }
    void dronePoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg){
        dronePose.pose = msg->pose;
        if(getFb&&getWp){
            X = waypoint.pose.position.x - dronePose.pose.position.x;
            Y = waypoint.pose.position.y - dronePose.pose.position.y;
            p = forbiddenCenter.pose.position.x - dronePose.pose.position.x;
            q = forbiddenCenter.pose.position.y - dronePose.pose.position.y;
            k = (X*p+Y*q)/(pow(X,2)+pow(Y,2));
            get_N();
            get_S();
//            get_G();
        }
    }
    void wpCallback(const geometry_msgs::PoseStamped::ConstPtr& msg){
        waypoint.header = msg->header;
        waypoint.pose = msg->pose;
        getWp = true;
    }
    void fbCallback(const geometry_msgs::PoseStamped::ConstPtr& msg){
        forbiddenCenter.pose = msg->pose;
        getFb = true;
    }
    void modeCallback(const std_msgs::Int8 &msg){
        mode = msg.data;
    }
    void get_N(){
        N.pose.position.x = k*X + dronePose.pose.position.x;
        N.pose.position.y = k*Y + dronePose.pose.position.y;
        N.header.frame_id = "map";
        N.pose.position.z = 0;
        N_pub.publish(N);
    }
    void get_S(){
       geometry_msgs::PoseStamped S;
       double dis_Fn = sqrt(pow(forbiddenCenter.pose.position.x - N.pose.position.x,2)+pow(forbiddenCenter.pose.position.y - N.pose.position.y,2));
       ROS_INFO("distance from F to N : %lf",dis_Fn);
       S.pose.position.x = (k - sqrt(abs(pow(r+s,2)-dis_Fn/**(pow(X,2)+pow(Y,2))-pow(X*q+Y*p,2)*/))/sqrt(pow(X,2)+pow(Y,2)))*X + dronePose.pose.position.x;
       S.pose.position.y = (k - sqrt(abs(pow(r+s,2)-dis_Fn/**(pow(X,2)+pow(Y,2))-pow(X*q+Y*p,2)*/))/sqrt(pow(X,2)+pow(Y,2)))*Y + dronePose.pose.position.y;
       S.header.frame_id = "map";
       S.pose.position.z = 0;
       S_pub.publish(S);
//    }
//    void get_G(){
        geometry_msgs::PoseStamped G1;
        geometry_msgs::PoseStamped G2;
        geometry_msgs::PoseStamped G;
        double d1, d2;
        ROS_INFO("theta : %lf \n",theta);

        G1.pose.position.x = cos(theta)*(S.pose.position.x-forbiddenCenter.pose.position.x) - sin(theta)*(S.pose.position.y-forbiddenCenter.pose.position.y) + forbiddenCenter.pose.position.x;
        G1.pose.position.y = sin(theta)*(S.pose.position.x-forbiddenCenter.pose.position.x) + cos(theta)*(S.pose.position.y-forbiddenCenter.pose.position.y) + forbiddenCenter.pose.position.y;
        G2.pose.position.x = cos(theta)*(S.pose.position.x-forbiddenCenter.pose.position.x) + sin(theta)*(S.pose.position.y-forbiddenCenter.pose.position.y) + forbiddenCenter.pose.position.x;
        G2.pose.position.y = -sin(theta)*(S.pose.position.x-forbiddenCenter.pose.position.x) + cos(theta)*(S.pose.position.y-forbiddenCenter.pose.position.y) + forbiddenCenter.pose.position.y;

        d1 = sqrt(pow(G1.pose.position.x-waypoint.pose.position.x,2) + pow(G1.pose.position.y-waypoint.pose.position.y,2));
        d2 = sqrt(pow(G2.pose.position.x-waypoint.pose.position.x,2) + pow(G2.pose.position.y-waypoint.pose.position.y,2));

        if(d1<=d2)
            G=G1;
        else G=G2;

        G.header.frame_id = "map";
        G.pose.position.z = waypoint.pose.position.z;
        if((abs(dis_Fn)<r) && (mode == AVOIDANCE))
        G_pub.publish(G);
        else
        G_pub.publish(waypoint);
    }

    void spin(){
      ros::Rate r(30);
        while(ros::ok()){
             //core();
            r.sleep();
            ros::spinOnce();
        }
    }
};

int main(int argc, char** argv){
    ros::init(argc,argv,"avoidance_operator");
    //OOP + dynamic allocation adapted.
    avoidance_operator *_avoidance_operator = new avoidance_operator();
    _avoidance_operator->spin();
    if(!ros::ok()){
        delete _avoidance_operator;
    }
}
