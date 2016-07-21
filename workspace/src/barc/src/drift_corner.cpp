#include "ros/ros.h"
#include"std_msgs/String.h"
#include "geometry_msgs/Vector3.h"
#include "data_service/TimeData.h"
#include "nav_msgs/Odometry.h"
#include <iostream>
#include <fstream>
#include "pid.h"
#include <vector>
#include <cmath>

ros::Duration t ;
ros::Time t0 ;

// current vehicle state 
float X,Y,yaw,vx,vy,yr;


// parameter definition 

  struct States
  {
    double X;
    double Y;
    double yaw;
    double vx;
    double vy;
    double yr;
  };

//initial yaw angle

float yaw0;
int  n_FL, n_FR;
int read_yaw0 = 0;
float odom_vx,odom_vy;
float odom_wz,odom_x,odom_y;
float odom_P;
float vx_est,vy_est,r_est,beta_est;
float m = 1.98,mu = 0.234,L_a,L_b,Iz;
float C_af,C_ar;
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

States vehicle_mdl(States pre_state,double dt,States noise,States mdl_err,double d_f,double F_xR)
{
  States nx_state;
  double dX,dY,dyaw,dvx,dvy,dyr;
  dX =  pre_state.vx*cos(pre_state.yaw) - pre_state.vy*sin(pre_state.yaw);
  dY =  pre_state.vx*sin(pre_state.yaw) + pre_state.vy*cos(pre_state.yaw);
  dyaw = pre_state.yr;
  dvx = F_xR/m;
  dvy = -(C_af+C_ar)/(m*pre_state.vx)*pre_state.vy;
  dvy = dvy + (L_b*C_ar-L_a*C_af)/(m*pre_state.vx)*pre_state.yr;
  dvy = dvy - pre_state.vx*pre_state.yr + C_af/m*d_f;
  dyr = (L_b*C_ar - L_a*C_af)/Iz/pre_state.vx*pre_state.vy;
  dyr = dyr - (pow(L_a,2)*C_af + pow(L_b,2)*C_ar)/Iz/pre_state.vx*pre_state.yr;
  dyr = dyr + L_a*C_af/Iz*d_f;

  nx_state.X = pre_state.X + dt*(dX + mdl_err.X + noise.X);
  nx_state.Y = pre_state.Y + dt*(dY + mdl_err.Y + noise.Y);
  nx_state.yaw = pre_state.yaw + dt*(dyaw + mdl_err.yaw + noise.yaw);
  nx_state.vx = pre_state.vx + dt*(dvx + mdl_err.vx + noise.vx);
  nx_state.vy = pre_state.vy + dt*(dvy + mdl_err.vy + noise.vy);
  nx_state.yr = pre_state.yr + dt*(dyr + mdl_err.yr + noise.yr);

  return nx_state;

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


  // obtain parameters

  std::vector<double> X_ol,Y_ol,yaw_ol,vx_ol,vy_ol,yr_ol;//open loop trajectory
  std::vector<double> d_f_ol,F_xR_ol; //open loop maneuver
  std::vector<double> X_err,Y_err,yaw_err,vx_err,vy_err,yr_err;//model error of open loop
  std::vector<double> K_d_f,K_F_xR; //close loop control policy

  //read open loop states and control inputs 

  std::ifstream file("/home/odroid/Desktop/test.csv");
  std::string line;
  while (std::getline(file,line))
  {
     std::istringstream iss(line);
     std::string result;
     std::vector<double>  ol_st_tr;
     while (std::getline(iss,result,','))
     {
       ol_st_tr.push_back(atof(result.c_str()));
     }
     X_ol.push_back(ol_st_tr[0]);
     Y_ol.push_back(ol_st_tr[1]);
     yaw_ol.push_back(ol_st_tr[2]);
     vx_ol.push_back(ol_st_tr[3]);
     vy_ol.push_back(ol_st_tr[4]);
     yr_ol.push_back(ol_st_tr[5]);
     K_d_f.push_back(ol_st_tr[6]);
     K_F_xR.push_back(ol_st_tr[7]);
  }

  // read open loop model error and close loop control policy
  std::ifstream file1("/home/odroid/Desktop/model_tro_poli.csv");
  std::string line1;
  while (std::getline(file1,line1))
  {
     std::istringstream iss(line1);
     std::string result;
     std::vector<double>  model_tro_poli;
     while (std::getline(iss,result,','))
     {
       model_tro_poli.push_back(atof(result.c_str()));
     }
     X_err.push_back(model_tro_poli[0]);
     Y_err.push_back(model_tro_poli[1]);
     yaw_err.push_back(model_tro_poli[2]);
     vx_err.push_back(model_tro_poli[3]);
     vy_err.push_back(model_tro_poli[4]);
     yr_err.push_back(model_tro_poli[5]);
     d_f_ol.push_back(model_tro_poli[6]);
     F_xR_ol.push_back(model_tro_poli[7]);
  }

   

  int N_state = 6,N_contol = 2;
  int ct_sp = 10; // 10 control step
  double dt = 0.1; // one control step time duration
  double Q[6] = {1,1,1,1,1,1};//cost function coeficient 
  double d_f_ol_man[ct_sp]; // open loop maneuver : steering angle
  double F_xR_ol_man[ct_sp]; // open loop maneuver : rear wheel force
  double d_f_cl_man[ct_sp]; // close loop maneuver : steering angle
  double F_xR_cl_man[ct_sp]; // close loop maneuver : rear wheel force

  
  States pre_ol[ct_sp+1],pre_cl[ct_sp+1]; //predicted states in the open loop maneuver
  States pre_des[ct_sp]; // the desired state trajectory
  States mdl_err[ct_sp];
  States noise; // vehicle states


  int motor_CMD,servo_CMD;
  float d_f;
  geometry_msgs::Vector3 ecu;
  ros::Rate r(50); // 50 Hz



  while(ros::ok())
  {

    // find the nearest point among the open loop 
    int count_nearest_pt = X_ol.size();
    int flag_nearest_pt = 0;
    double dis = 10000;
    for (int i = 0;i < count_nearest_pt;i++ )
    {
      double temp_dis = pow(pow(X-X_ol[i],2)+pow(Y-Y_ol[i],2)+pow(yaw-yaw_ol[i],2),0.5);
      if (temp_dis < dis)
      {
        flag_nearest_pt = i;
        dis = temp_dis;
      }
    }

    // compute the desired state trajectory
    for (int i = 0; i< ct_sp;i++)
    {
      pre_des[i].X =   X_ol[i + count_nearest_pt];
      pre_des[i].Y =   Y_ol[i + count_nearest_pt];
      pre_des[i].yaw = yaw_ol[i + count_nearest_pt];
      pre_des[i].vx =  vx_ol[i + count_nearest_pt];
      pre_des[i].vy =  vy_ol[i + count_nearest_pt];
      pre_des[i].yr =  yr_ol[i + count_nearest_pt];
    }
    
   // model error 

    for (int i = 0; i< ct_sp; i++)
    {
      mdl_err[i].X =   X_err[i+flag_nearest_pt];
      mdl_err[i].Y =   Y_err[i+flag_nearest_pt];
      mdl_err[i].yaw = yaw_err[i+flag_nearest_pt];
      mdl_err[i].vx =  vx_err[i+flag_nearest_pt];
      mdl_err[i].vy =  vy_err[i+flag_nearest_pt];
      mdl_err[i].yr =  yr_err[i+flag_nearest_pt];
    }
    // design the open loop maneuver

    for (int i = 0; i < ct_sp;i++)
    {
      d_f_ol_man[i] = d_f_ol[i + flag_nearest_pt];
      F_xR_ol_man[i] = F_xR_ol[i + flag_nearest_pt];
    }

    

    // states envolve in open loop maneuver

    pre_ol[0].X = X;
    pre_ol[0].Y = Y;
    pre_ol[0].yaw = yaw;
    pre_ol[0].vx = vx;
    pre_ol[0].vy = vy;
    pre_ol[0].yr = yr;

    for (int i = 1; i < ct_sp;i++)
    {
      pre_ol[i] = vehicle_mdl(pre_ol[i-1],dt,noise,mdl_err[i-1],d_f_ol_man[i-1],F_xR_ol_man[i-1]);
    }


    
    // design the close loop maneuver and the states envolve 
    
    pre_cl[0].X = X;
    pre_cl[0].Y = Y;
    pre_cl[0].yaw = yaw;
    pre_cl[0].vx = vx;
    pre_cl[0].vy = vy;
    pre_cl[0].yr = yr;

    for (int i = 1; i < ct_sp;i++)
    {
      d_f_cl_man[i-1] =  K_d_f[(i-1) + N_state*count_nearest_pt]*pre_cl[i-1].vx + \
                         K_d_f[(i-1) + N_state*count_nearest_pt + 1]*pre_cl[i-1].vy + \
                         K_d_f[(i-1) + N_state*count_nearest_pt + 2]*pre_cl[i-1].yr + \
                         K_d_f[(i-1) + N_state*count_nearest_pt + 3]*pre_cl[i-1].X + \
                         K_d_f[(i-1) + N_state*count_nearest_pt + 4]*pre_cl[i-1].Y + \
                         K_d_f[(i-1) + N_state*count_nearest_pt + 5]*pre_cl[i-1].yaw ;
      F_xR_cl_man[i-1] = K_F_xR[(i-1) + N_state*count_nearest_pt]*pre_cl[i-1].vx + \
                         K_F_xR[(i-1) + N_state*count_nearest_pt + 1]*pre_cl[i-1].vy + \
                         K_F_xR[(i-1) + N_state*count_nearest_pt + 2]*pre_cl[i-1].yr + \
                         K_F_xR[(i-1) + N_state*count_nearest_pt + 3]*pre_cl[i-1].X + \
                         K_F_xR[(i-1) + N_state*count_nearest_pt + 4]*pre_cl[i-1].Y + \
                         K_F_xR[(i-1) + N_state*count_nearest_pt + 5]*pre_cl[i-1].yaw ;
     pre_cl[i] = vehicle_mdl(pre_cl[i-1],dt,noise,mdl_err[i-1],d_f_cl_man[i-1],F_xR_cl_man[i-1]);  
    }


    // compute the cost function of open loop
  

   double cost_fn_ol = 0,cost_fn_cl = 0;

   for (int i = 0; i < ct_sp;i++)
   {
     double del_vx  = pre_ol[i+1].vx - pre_des[i].vx;
     double del_vy  = pre_ol[i+1].vy - pre_des[i].vy;
     double del_yr  = pre_ol[i+1].yr - pre_des[i].yr;
     double del_X   = pre_ol[i+1].X - pre_des[i].X;
     double del_Y   = pre_ol[i+1].Y - pre_des[i].Y;
     double del_yaw = pre_ol[i+1].yaw - pre_des[i].yaw;

     double temp_cost = del_vx*Q[0]*del_vx + del_vy*Q[1]*del_vy + del_yr*Q[2]*del_yr +\
                        del_X*Q[3]*del_X + del_Y*Q[4]*del_Y + del_yaw*Q[5]*del_yaw;
     double cost_fn_ol = cost_fn_ol + temp_cost;
   }

    com_pub.publish(ecu);
    
    ros::spinOnce();
    r.sleep();
  }
  return 0;
   
}
