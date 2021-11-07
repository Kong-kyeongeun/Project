#include "ros/ros.h"
#include "math.h"

#define PI 3.1415926535897
void alignment(bool rl_direction , int length) //dirction 왼 0 오 1
{
    double L.length;
    double R.length;
      if(direction){ //오른쪽 공간이 더큼

    int adj_length = R.length-(L.length + R.length)/2
    double r; // r은 핸들 꺾은 각도에 따른 회전 반지름
    double angle; //치우쳐진 쪽으로 이동하기 위해 핸들을 돌린후 진행햐였을때 원에서 진행한 각도
    double time; //시간 (angle까지 도달하는데 걸리는)
    double v; //차속력(선속도)
    angle = acos((r-adj_length/2)/r); //x = r - rcos(angle)
    time = r*angle/v //angle = v/r * t

    Right(double v,double time, bool fb_dirction, double angle) //속력 v로 직진혹은 후진 오른쪽으로 angle만큼 틀고 time 만큼 가겠다.
    Left(double v, double time, bool fb_dirction, double -angle) //속력 v로 직진혹은 후진 왼쪽으로 angle만큼 틀고 time 만큼 가겠다.     
    }
    if(!direction){ //왼쪽 공간이 더큼

    int adj_length = l.length-(L.length + R.length)/2
    double r; // r은 핸들 꺾은 각도에 따른 회전 반지름
    double angle; //치우쳐진 쪽으로 이동하기 위해 핸들을 돌린후 진행햐였을때 원에서 진행한 각도
    double time; //시간 (angle까지 도달하는데 걸리는)
    double v; //차속력(선속도)
    angle = acos((r-adj_length/2)/r); //x = r - rcos(angle)
    time = r*angle/v //angle = v/r * t
    
    Left(double v, double time, bool fb_dirction, double -angle) //속력 v로 직진혹은 후진 왼쪽으로 angle만큼 틀고 time 만큼 가겠다.
    Right(double v,double time, bool fb_dirction, double angle) //속력 v로 직진혹은 후진 오른쪽으로 angle만큼 틀고 time 만큼 가겠다.
           
    }

int main()
{
double L.length;
double R.length;
double B.length;



}
}

