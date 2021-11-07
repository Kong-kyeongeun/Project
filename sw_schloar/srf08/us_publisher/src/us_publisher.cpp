#include "ros/ros.h"// ROS 기본 헤더 파일
#include "us/Msgus.h"// MsgTutorial 메시지 파일 헤더(빌드 후 자동 생성 됨)
#include <stdio.h>
#include <string.h>
#include <wiringPi.h>
#include <wiringPiI2C.h>

int main(int argc, char **argv) // 노드 메인 함수
{
ros::init(argc, argv, "us"); // 노드명 초기화
ros::NodeHandle nh;// ROS 시스템과 통신을 위한 노드 핸들 선언

ros::Publisher us_pub = nh.advertise<us::Msgus>("us_msg", 100);
// 루프주기를설정한다. "10" 이라는것은10Hz를말하는것으로0.1초간격으로반복된다

ros::Rate loop_rate(10);
// MsgTutorial메시지파일형식으로msg라는메시지를선언

us::Msgus msg;
// 메시지에사용될변수선언

while (ros::ok())
 {
int rtc;
float r;
rtc = wiringPiI2CSetup(0x73);
delay(100);
wiringPiI2CWriteReg16(rtc,0x00,0x51);
delay(100);
r = wiringPiI2CReadReg16(rtc, 2) >> 8;
delay(100);
msg.distance = r;
ROS_INFO("distance= %f", msg.distance); // data 메시지를표시한다
us_pub.publish(msg);// 메시지를발행한다
loop_rate.sleep();// 위에서정한루프주기에따라슬립에들어간다
 }

return 0;
}
