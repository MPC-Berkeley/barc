#include <ros/ros.h>
#include <barc_cpp/ECU.h>
#include <barc_cpp/Encoder.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include <tf/tf.h>

#include <tuple>               // for returning multiple values

#include "ekf.hpp"
#include "system_models.hpp"

#include <cmath>

#include <vector>
#include <algorithm>

#define PI 3.14159
using namespace std;

// input variables
double d_f = 0;
double FxR = 0;

// raw measurement variables
double yaw_prev = 0;
double roll=0, pitch=0, yaw=0;
double a_x=0,  a_y=0,   a_z=0;
double w_x=0,  w_y=0,   w_z=0;

// from encoder
double v_x_enc = 0;
double t0 = ros::Time::now().toSec();
double n_FL = 0;   // counts in the front left tire
double n_FR = 0;   // counts in the front right tire
double n_FL_prev = 0;
double n_FR_prev = 0;
double r_tire = 0.04; // radius from tire center to perimeter along magnets [m]
double dx_qrt = 2.0*PI*r_tire/4.0; // distance along quarter tire edge;

void ecu_callback(const barc_cpp::ECU::ConstPtr& msg){
  FxR = msg->motor;   // input motor force [N]
  d_f = msg->servo;  // input steering angle [rad]
}

void imu_callback(const sensor_msgs::Imu::ConstPtr& msg){
  geometry_msgs::Quaternion ori = msg->orientation;

  tf::Quaternion q(ori.x, ori.y, ori.z, ori.w);
  tf::Matrix3x3 m(q);
  m.getRPY(roll, pitch, yaw);  // get Roll Pitch Yaw

  yaw_prev = yaw;

  w_x = msg->angular_velocity.x;
  w_y = msg->angular_velocity.y;
  w_z = msg->angular_velocity.z;
  a_x = msg->linear_acceleration.x;
  a_y = msg->linear_acceleration.y;
  a_z = msg->linear_acceleration.z;
}

void enc_callback(const barc_cpp::Encoder::ConstPtr& msg){
  n_FL = msg->FL;
  n_FR = msg->FR;

  double tf = ros::Time::now().toSec();
  double dt;
  double dt_min = 0.2;
  double v_FL, v_FR;

  dt = tf - t0;

  if(dt >= dt_min){
    v_FL = float(n_FL - n_FL_prev)* dx_qrt/dt;
    v_FR = float(n_FR - n_FR_prev)* dx_qrt/dt;

    v_x_enc = (v_FL + v_FR) / 2.0*cos(d_f);

    n_FL_prev = n_FL;
    n_FR_prev = n_FR;
    t0 = ros::Time::now().toSec();
  }
}

int main(int argc, char** argv){
  ros::init(argc, argv, "state_estimation_DynBkMdl_node");
  ros::NodeHandle n;

  // Initialize topics
  ros::Subscriber imu_sub = n.subscribe("imu/data",1,imu_callback);
  ros::Subscriber enc_sub = n.subscribe("encoder",1,enc_callback);
  ros::Subscriber ecu_sub = n.subscribe("enu",1,ecu_callback);

  ros::Publisher state_pub = n.advertise<geometry_msgs::Vector3>("state_estimate",10);

  double L_a, L_b, m, I_z, dt_vx;
  double B, C, mu;
  double a0, Ff, q_std, r_std, v_x_min;

  n.getParam("L_a", L_a);
  n.getParam("L_b", L_b);
  n.getParam("m", m);
  n.getParam("I_z", I_z);

  n.getParam("state_estimation/dt_v_enc", dt_vx);

  n.getParam("tire_model/B", B);
  n.getParam("tire_model/C", C);
  n.getParam("tire_model/mu", mu);

  n.getParam("air_draf_coeff", a0);
  n.getParam("friction", Ff);

  n.getParam("state_estimation/q_std", q_std);
  n.getParam("state_estimation/r_std", r_std);
  n.getParam("state_estimation/v_x_min", v_x_min);

  Eigen::MatrixXf vhMdl(1,4);
  Eigen::MatrixXf TrMdl(1,3); // assume TrMdlFront == TrMdlRear
  vhMdl << L_a, L_b, m, I_z;
  TrMdl << B , C, mu;

  double loop_rate = 50;
  double dt = 1.0 / loop_rate;
  double t0 = ros::Time::now().toSec();

  ros::Rate rate(loop_rate);

  Eigen::MatrixXf z_EKF(1,3);
  z_EKF << 1.0, 0.0, 0.0;

  // create diagonal matrix P Q R
  Eigen::Matrix3f P;
  Eigen::Matrix3f Q;
  Eigen::Matrix2f R;
  P << 1,0,0,
       0,1,0,
       0,0,1;
  Q << 1,0,0,
       0,1,0,
       0,0,1;
  R << 1,0,
       0,1;

  double v_x, v_y, r;

  while ( ros::ok() ){
    v_x = z_EKF(0,0);
    v_y = z_EKF(0,1);
    r   = z_EKF(0,2);

    geometry_msgs::Vector3 state_msg;
    state_msg.x = v_x;
    state_msg.y = v_y;
    state_msg.z = r;

    // state_estimate로 퍼블리시
    state_pub.publish(state_msg);

    if (v_x_enc > v_x_min) {
      Eigen::MatrixXf y(1,2);
      Eigen::MatrixXf u(1,2);
      Eigen::MatrixXf F_ext(1,2);
      y <<     v_x_enc,  w_z;
      u <<     d_f,      FxR;
      F_ext << a0,       Ff;

      std::vector<Eigen::MatrixXf> args;
      args.push_back(u);
      args.push_back(vhMdl);
      args.push_back(TrMdl);  // assume TrMdlFront == TrMdlRear
      args.push_back(F_ext);

      std::tie(z_EKF, P) = ekf(f_3s, z_EKF, P, h_3s, y, Q, R, args, dt);
    }
    else{
      z_EKF(0,0) = v_x_enc;
      z_EKF(0,2) = w_z;
    }
    rate.sleep();
  }

  return 0;
}
