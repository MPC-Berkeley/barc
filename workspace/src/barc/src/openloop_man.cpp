#include "ros/ros.h"
#include"std_msgs/String.h"
#include "geometry_msgs/Vector3.h"
#include "data_service/TimeData.h"
#include "nav_msgs/Odometry.h"
#include "barc/ECU.h"
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

void state_Callback(const geometry_msgs::Vector3 msg)
{
  vx_est = msg.x;
  vy_est = msg.y;
  r_est= msg.z;
  //beta_est =atan(vy_est/vx_est);
}

void odom_Callback(const nav_msgs::Odometry  msg)
 {
  float temp = msg.twist.twist.linear.x/odom_P;
  if (abs(temp - odom_vx) > 1) // donot updat when the state change large
  {
  }
  else{
  odom_vx = msg.twist.twist.linear.x/odom_P;
  odom_vy = msg.twist.twist.linear.y/odom_P;
  odom_wz = msg.twist.twist.angular.z/odom_P;
  odom_x = msg.pose.pose.position.x/odom_P;
  odom_y = msg.pose.pose.position.y/odom_P;
  }
}

void gps_Callback(const geometry_msgs::Vector3  msg)
{
  X = msg.x;
  Y = msg.y;
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


  time_t time_now;
  time(&time_now);
  std::string s = ctime(&time_now); 
  std::ofstream Data (("/home/odroid/Data/Drift_corner/"+s+".csv").c_str());
  Data << "t,vx,vy,yawrate,X,Y,yaw,steeing,F_xR"<<std::endl;
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
      d_f = 0;

    }
    else if (t.toSec() < t_hold + t_straight + t_turn)  // make a turn 
    {
      F_xR = F_xR_turn;
      d_f = d_f_turn;

    }
    else if (t.toSec() < t_hold + t_straight + t_turn + t_counter)      //introduce counter_steering 
    {
    F_xR = F_xR_counter;
    d_f = d_f_counter;
    }
    else if (t.toSec() < t_hold + t_straight + t_turn + t_counter + t_recover)//         go back to straight
    {
      F_xR = F_xR_recover;
      d_f = 0;
    }
    else
    {
      F_xR = 0;
      d_f = 0;
    }

    Data << t << "," << vx << "," << vy << "," << std::endl;
    //std::cout << "d_f :" << d_f <<std::endl;
    //std::cout << "F_xR :" << F_xR <<std::endl;


    

    ecu.motor = F_xR;
    ecu.servo = d_f*pi/180; //degree -> rad
    com_pub.publish(ecu);
    
    r.sleep();
  }
  return 0;
   
}
