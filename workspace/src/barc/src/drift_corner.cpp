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
#include "barc/ECU.h"
#include "barc/six_states.h"


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
float m = 1.98,L_a = 0.125,L_b = 0.125,Iz = 0.24;
float C_af = -1.673,C_ar = -1.673;
float pi = 3.1415926;
int motorCMD_LQR,servoCMD_LQR;
int max_steer,min_steer,max_motor,min_motor;




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
  ros::NodeHandle state_est;
  ros::NodeHandle command;
  ros::NodeHandle param;

  ros::Subscriber state_sub = state_est.subscribe("/state_estimate",100,state_Callback);
  ros::Publisher com_pub = command.advertise<barc::ECU>("ecu",100);


  // obtain parameters

  std::vector<double> X_ol,Y_ol,yaw_ol,vx_ol,vy_ol,yr_ol;//open loop trajectory
  std::vector<double> d_f_ol,F_xR_ol; //open loop maneuver
  std::vector<double> X_err,Y_err,yaw_err,vx_err,vy_err,yr_err;//model error of open loop
  std::vector<double> K_d_f,K_F_xR; //close loop control policy

  //read open loop states and control inputs 

  std::ifstream file("/home/odroid/barc/workspace/src/barc/src/openloop_state_maneuver.csv");
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
     d_f_ol.push_back(ol_st_tr[6]);
     F_xR_ol.push_back(ol_st_tr[7]);
  }

  // read open loop model error
  std::ifstream file1("/home/odroid/barc/workspace/src/barc/src/model_error.csv");
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
  }

   
  // read close loop maneuver
  std::ifstream file2("/home/odroid/barc/workspace/src/barc/src/closeloop_maneuver.csv");
  std::string line2;
  while (std::getline(file2,line2))
  {
     std::istringstream iss(line2);
     std::string result;
     std::vector<double>  closeloop;
     while (std::getline(iss,result,','))
     {
       closeloop.push_back(atof(result.c_str()));
     }
     K_d_f.push_back(closeloop[0]);
     K_F_xR.push_back(closeloop[1]);
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
  

  // nosie 

  noise.X = 0;
  noise.Y = 0;
  noise.yaw = 0;
  noise.vx = 0.1;
  noise.vy = 0.05;
  noise.yr = 0.1;

  barc::ECU ecu;
  int closeloop_flag = 0;
  ros::Rate r(50); // 50 Hz

  X = -2.36;
  Y = 3.73;
  yaw = -0.0019;
  vx = 0.01;
  vy = 0;
  yr = 0;


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
     
    std::cout << "nearest point : "<<flag_nearest_pt<<std::endl;

    // compute the desired state trajectory
    for (int i = 0; i< ct_sp;i++)
    {
      pre_des[i].X =   X_ol[i + flag_nearest_pt];
      pre_des[i].Y =   Y_ol[i + flag_nearest_pt];
      pre_des[i].yaw = yaw_ol[i + flag_nearest_pt];
      pre_des[i].vx =  vx_ol[i + flag_nearest_pt];
      pre_des[i].vy =  vy_ol[i + flag_nearest_pt];
      pre_des[i].yr =  yr_ol[i + flag_nearest_pt];

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

    for (int i = 1; i < ct_sp + 1;i++)
    {
      pre_ol[i] = vehicle_mdl(pre_ol[i-1],dt,noise,mdl_err[i-1],d_f_ol_man[i-1],F_xR_ol_man[i-1]);
      //std::cout << "pre_ol" << i << ":" <<pre_ol[i].vy<<std::endl;
    }


    
    // design the close loop maneuver and the states envolve 
    
    pre_cl[0].X = X;
    pre_cl[0].Y = Y;
    pre_cl[0].yaw = yaw;
    pre_cl[0].vx = vx;
    pre_cl[0].vy = vy;
    pre_cl[0].yr = yr;

    for (int i = 1; i < ct_sp + 1;i++)
    {
      d_f_cl_man[i-1] =  K_d_f[(i-1) + N_state*count_nearest_pt]*pre_cl[i-1].vx + \
                         K_d_f[(i-1) + N_state*count_nearest_pt + 1]*pre_cl[i-1].vy + \
                         K_d_f[(i-1) + N_state*count_nearest_pt + 2]*pre_cl[i-1].yr + \
                         K_d_f[(i-1) + N_state*count_nearest_pt + 3]*pre_cl[i-1].X + \
                         K_d_f[(i-1) + N_state*count_nearest_pt + 4]*pre_cl[i-1].Y + \
                         K_d_f[(i-1) + N_state*count_nearest_pt + 5]*pre_cl[i-1].yaw + d_f_ol_man[i-1];
      F_xR_cl_man[i-1] = K_F_xR[(i-1) + N_state*count_nearest_pt]*pre_cl[i-1].vx + \
                         K_F_xR[(i-1) + N_state*count_nearest_pt + 1]*pre_cl[i-1].vy + \
                         K_F_xR[(i-1) + N_state*count_nearest_pt + 2]*pre_cl[i-1].yr + \
                         K_F_xR[(i-1) + N_state*count_nearest_pt + 3]*pre_cl[i-1].X + \
                         K_F_xR[(i-1) + N_state*count_nearest_pt + 4]*pre_cl[i-1].Y + \
                         K_F_xR[(i-1) + N_state*count_nearest_pt + 5]*pre_cl[i-1].yaw + F_xR_ol_man[i-1];
      std::cout << "d_f_cl" << i << ": " << d_f_cl_man[i-1]<<std::endl;
      std::cout << "F_xR_cl" << i << ": " << F_xR_cl_man[i-1]<<std::endl;
      pre_cl[i] = vehicle_mdl(pre_cl[i-1],dt,noise,mdl_err[i-1],d_f_cl_man[i-1],F_xR_cl_man[i-1]);  

      std::cout << "pre_cl.X" << i << ": " << pre_cl[i].X<<std::endl;
    }


    // compute the cost function of open loop and close loop maneuver
  

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
     cost_fn_ol = cost_fn_ol + temp_cost;
   }
   for (int i = 0; i < ct_sp;i++)
   {
     double del_vx  = pre_cl[i+1].vx - pre_des[i].vx;
     double del_vy  = pre_cl[i+1].vy - pre_des[i].vy;
     double del_yr  = pre_cl[i+1].yr - pre_des[i].yr;
     double del_X   = pre_cl[i+1].X - pre_des[i].X;
     double del_Y   = pre_cl[i+1].Y - pre_des[i].Y;
     double del_yaw = pre_cl[i+1].yaw - pre_des[i].yaw;

     double temp_cost = del_vx*Q[0]*del_vx + del_vy*Q[1]*del_vy + del_yr*Q[2]*del_yr +\
                        del_X*Q[3]*del_X + del_Y*Q[4]*del_Y + del_yaw*Q[5]*del_yaw;
     cost_fn_cl = cost_fn_cl + temp_cost;
   }

    // decide open loop or closed loop
    
    if (cost_fn_cl > cost_fn_ol)
    {
      closeloop_flag = 0;
      ecu.motor = F_xR_ol_man[0];
      ecu.servo = d_f_ol_man[0];
    }
    else
    {
      closeloop_flag = 1;
      ecu.motor = F_xR_cl_man[0];
      ecu.servo = d_f_cl_man[0];
    }
    com_pub.publish(ecu);
    
    ros::spinOnce();
    r.sleep();
  }
  return 0;
   
}
