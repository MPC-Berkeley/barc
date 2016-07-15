#include "ros/ros.h"
#include"std_msgs/String.h"
#include "geometry_msgs/Vector3.h"
#include "data_service/TimeData.h"
#include "nav_msgs/Odometry.h"
#include <iostream>
#include <fstream>
#include "pid.h"
#include <vector>

ros::Duration t ;
ros::Time t0 ;
int  n_FL, n_FR;
int read_yaw0 = 0;
float odom_vx,odom_vy, yr,yaw,yaw0;
float odom_wz,odom_x,odom_y;
float odom_P;
float vx_est,vy_est,r_est,beta_est;
float m = 1.98,mu = 0.234, a0 =0.1308, Ff=0.1711,L_a = 0.125;
float B = 7.4,C=1.25;
float pi = 3.1415926;
int motorCMD_LQR,servoCMD_LQR;
int max_steer,min_steer,max_motor,min_motor;


int  angle_2_servo(float x)
{
    int  u   = 92.0558 + 1.8194*x  - 0.0104*x*x;
    return u;
}

int  servo_2_angle(float x)
{
  int d_f  = -(39.2945 - 0.3013*x  - 0.0014*x*x);
  return d_f;
}
void enc_Callback(const geometry_msgs::Vector3  msg)
{
  n_FL = msg.x;
  n_FR = msg.y;
}

void imu_Callback(const data_service::TimeData  msg)
{
  if(read_yaw0 == 0)
  {
    yaw0 = msg.value[2];
    read_yaw0 = 1;
  }
  else
  {
  yr = msg.value[8];
  yaw = msg.value[2];
  }
}
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

int main(int argc, char **argv)
{
 
  ros::init(argc, argv, "drift_corner");
  ros::NodeHandle enc;
  ros::NodeHandle imu;
  ros::NodeHandle odom;
  ros::NodeHandle state_est;
  ros::NodeHandle command;
  ros::NodeHandle param;

  ros::Subscriber enc_sub = enc.subscribe("enc_data", 1000, enc_Callback);
  ros::Subscriber imu_sub = imu.subscribe("imu_data", 1000, imu_Callback);
  ros::Subscriber odom_sub = odom.subscribe("mono_odometer/odometry", 1000, odom_Callback);
  ros::Subscriber state_sub = state_est.subscribe("state_estimate",100,state_Callback);
  ros::Publisher com_pub = command.advertise<geometry_msgs::Vector3>("ecu_cmd",100);


  // ontain parameters

   std::vector<double> Y_ol;

  int motor_CMD,servo_CMD;
  float d_f;
  geometry_msgs::Vector3 ecu;
  ros::Rate r(50); // 50 Hz



  while(ros::ok())
  {

    ros::spinOnce();
    motor_CMD = 90;
    d_f = 0;
    servo_CMD = angle_2_servo(d_f); 
       

    ecu.x = motor_CMD;
    ecu.y = servo_CMD;
    ecu.z = d_f;
    com_pub.publish(ecu);
    
    r.sleep();
  }
  return 0;
   
}
