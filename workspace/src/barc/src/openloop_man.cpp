#include "ros/ros.h"
#include"std_msgs/String.h"
#include "geometry_msgs/Vector3.h"
#include "data_service/TimeData.h"
#include "nav_msgs/Odometry.h"
#include "barc/ECU.h"
#include "barc/six_states.h"
#include <iostream>
#include <fstream>
#include "pid.h"
#include <ctime>

ros::Duration t ;
ros::Time t0 ;
int  n_FL, n_FR;
int read_yaw0 = 0;
float odom_vx,odom_vy,yaw0;
float odom_wz,odom_x,odom_y;
float odom_P;
float vx_est,vy_est,r_est,beta_est;
int activate_LQR = 0;
float P[3][3] = {{2.8678,-0.7167,-0.98},{-0.7167,4.4277,-0.0443},{-0.98,-0.0443,1.4411}};
float K[2][3] = {{-1.1052,-1.9455,0.5623},{2.0218,-2.2139,-1.1233}};
float resual_ref=0.1721*5;
float vx_ref = 1,beta_ref = -0.50,r_ref = 1.76;
float F_yf_ref = 1.7153,F_xr_ref = 1.4377;
float m = 1.98,mu = 0.234, a0 =0.1308, Ff=0.1711,L_a = 0.125;
float B = 7.4,C=1.25;
float pi = 3.1415926;
int motorCMD_LQR,servoCMD_LQR;
int max_steer,min_steer,max_motor,min_motor;

float vx,vy,yr,X,Y,yaw;

void state_Callback(const barc::six_states msg)
{
  if (read_yaw0 == 0 )
  {
    yaw0 = msg.yaw;
    read_yaw0 = 1;
  }
  else 
  {
    X = msg.X;
    Y = msg.Y;
    yaw = msg.yaw;
    vx = msg.vx;
    vy = msg.vy;
    yr= msg.yr;
  }
}


int main(int argc, char **argv)
{
 
  ros::init(argc, argv, "LQR_odo");
  ros::NodeHandle state_est;
  ros::NodeHandle command;
  ros::NodeHandle param;

  ros::Subscriber state_sub = state_est.subscribe("/state_estimate",100,state_Callback);
  ros::Publisher com_pub = command.advertise<barc::ECU>("ecu",100);
  float d_f,F_xR;
  barc::ECU ecu;
  ros::Rate r(50); // 50 Hz
  t0 = ros::Time::now();


  float t_hold,t_straight,t_turn,t_counter,t_recover;
  float F_xR_straight,F_xR_turn,F_xR_counter,F_xR_recover;
  float d_f_turn,d_f_counter;
  PID pid(0.02,30,-30,50,5,5);
  param.getParam("/openloop_man/t_hold",t_hold) ;// time duration of stay still
  param.getParam("/openloop_man/t_straight",t_straight) ;// time duration of go straight 
  param.getParam("/openloop_man/t_turn",t_turn) ;// time duration of turning
  param.getParam("/openloop_man/t_counter",t_counter) ;// time duration of counter-steer
  param.getParam("/openloop_man/t_recover",t_recover) ;// time duration of go back to straight
  param.getParam("/openloop_man/F_xR_straight",F_xR_straight) ;// rear wheel force to go straight
  param.getParam("/openloop_man/F_xR_turn",F_xR_turn) ;// rear wheel force to turn
  param.getParam("/openloop_man/F_xR_counter",F_xR_counter) ;// real wheel force when introducing counter-steering
  param.getParam("/openloop_man/F_xR_recover",F_xR_recover) ;// real wheel force when going back to straight
  param.getParam("/openloop_man/d_f_turn",d_f_turn) ;// steering angle when introducing steering
  param.getParam("/openloop_man/d_f_counter",d_f_counter) ;// steering angle when introducing counter-steering

  while(ros::ok())
  {

    ros::spinOnce();
    t = ros::Time::now()-t0;
    if( t.toSec() < t_hold)  // stay still
    {
      F_xR = 0;
      d_f = 0;
    }
    else if (t.toSec() < t_hold + t_straight) // go straight 
    {
      F_xR = F_xR_straight;
      d_f = pid.calculate(yaw0,yaw);
      //std::cout << "yaw0 :" << yaw0 <<"yaw" <<yaw <<std::endl;
      //std::cout << "d_yaw:"<< (yaw - yaw0)*180/pi<<std::endl;
      //std::cout << "d_f:" << d_f<<std::endl;
      

    }
    else if (t.toSec() < t_hold + t_straight + t_turn)  // make a turn 
    {
      F_xR = F_xR_turn;
      d_f = d_f_turn;

    }
    else if (t.toSec() < t_hold + t_straight + t_turn + t_counter)      //introduce counter_steering 
    {
      F_xR = F_xR_counter +  (1.4 - F_xR_counter)*(t.toSec()-t_hold-t_straight-t_turn)/t_counter;
      d_f = d_f_counter +   (0 - d_f_counter)*(t.toSec()-t_hold-t_straight-t_turn)/t_counter;
    }
    else
    {
      F_xR = 1.2;
      d_f = pid.calculate(-pi,yaw);
    }

    //std::cout << "d_f :" << d_f <<std::endl;
    //std::cout << "F_xR :" << F_xR <<std::endl;


    
    ecu.motor = F_xR;
    ecu.servo = d_f*pi/180; //degree -> rad
    com_pub.publish(ecu);
    
    r.sleep();
  }
  return 0;
   
}
