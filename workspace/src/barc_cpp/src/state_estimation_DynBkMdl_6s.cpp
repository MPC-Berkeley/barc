#include <ros/ros.h>
#include <barc_cpp/ECU.h>
#include <barc_cpp/Encoder.h>
// ed: msg added for 6 states
#include <barc_cpp/six_states.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include <arduino_sub_pub/msgs_type.h>
#include <tf/tf.h>

#include <tuple>               // for returning multiple values

// ed: name change for 6 states
#include "ekf_6s.hpp"
#include "system_models_6s.hpp"

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
double yaw0 = 0;
double yaw_local = 0;
bool read_yaw0 = false;
double roll=0, pitch=0, yaw=0;
double a_x=0,  a_y=0,   a_z=0;
double w_x=0,  w_y=0,   w_z=0;

// from encoder
double v_x_enc = 0;
//double t0 = ros::Time::now().toSec();
double t0;
double n_FL = 0;   // counts in the front left tire
double n_FR = 0;   // counts in the front right tire
double n_FL_prev = 0;
double n_FR_prev = 0;

// variable added for 6 states
double X_gps;
double Y_gps;
double v_x_optic;
double v_y_optic;


void ecu_callback(const barc_cpp::ECU::ConstPtr& msg){
  FxR = msg->motor;   // input motor force [N]
  d_f = msg->servo;   // input steering angle [rad]
}

void imu_callback(const sensor_msgs::Imu::ConstPtr& msg){
  geometry_msgs::Quaternion ori = msg->orientation;

  tf::Quaternion q(ori.x, ori.y, ori.z, ori.w);
  tf::Matrix3x3 m(q);
  m.getRPY(roll, pitch, yaw);  // get Roll Pitch Yaw

  if (!read_yaw0){
  	read_yaw0 = true;
	yaw_prev = yaw;
	yaw0 = yaw;
  }

  // python unwrap code conversion
  if(yaw >= 2*PI) yaw = 0;

  yaw_prev  = yaw;
  yaw_local = yaw - yaw0;

  w_x = msg->angular_velocity.x;
  w_y = msg->angular_velocity.y;
  w_z = msg->angular_velocity.z;
  a_x = msg->linear_acceleration.x;
  a_y = msg->linear_acceleration.y;
  a_z = msg->linear_acceleration.z;
}

void enc_callback(const arduino_sub_pub::msgs_type::ConstPtr& msg){
  double v_FL, v_FR;
  double tf = ros::Time::now().toSec();
  double dt;
  double dt_min = 0.2;

  v_FL = msg->rpm1;
  v_FR = msg->rpm2;

  dt = tf - t0;

  if(dt >= dt_min){
    v_x_enc = (v_FL + v_FR) / 2.0*cos(d_f);
    t0 = ros::Time::now().toSec();
  }
}

// function added for 6 states
void gps_callback(const geometry_msgs::Vector3::ConstPtr& msg){
	X_gps = msg->x / 100.0;
	Y_gps = msg->y / 100.0;
}

int main(int argc, char** argv){
  ros::init(argc, argv, "state_estimation_DynBkMdl_node");
  ros::NodeHandle n;

  // Initialize topics
  ros::Subscriber imu_sub = n.subscribe("imu/data",1,imu_callback);
  ros::Subscriber enc_sub = n.subscribe("hall_parsing_data",1,enc_callback);
  ros::Subscriber ecu_sub = n.subscribe("ecu",1,ecu_callback);

  // subscriber added for 6 states
  ros::Subscriber gps_sub = n.subscribe("indoor_gps", 1, gps_callback);


  ros::Publisher state_pub = n.advertise<barc_cpp::six_states>("state_estimate",10);

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

  Eigen::MatrixXf z_EKF(1,6);
  z_EKF << 0.0, 0.0, 0.0, 1.0, 0.0, 0.0;

  // create diagonal matrix P Q R
  Eigen::MatrixXf P(6,6);
  Eigen::MatrixXf Q(6,6);
  Eigen::MatrixXf R(6,6);
  P << 1,0,0,0,0,0,
       0,1,0,0,0,0,
       0,0,1,0,0,0,
	   0,0,0,1,0,0,
	   0,0,0,0,1,0,
	   0,0,0,0,0,1;
  Q << 0.01,0,0,0,0,0,
       0,0.01,0,0,0,0,
       0,0,0.01,0,0,0,
	   0,0,0,100000.0,0,0,
	   0,0,0,0,100000.0,0,
	   0,0,0,0,0,1.0;
  R << 0.01,0,0,0,0,0,
       0,0.01,0,0,0,0,
	   0,0,0.01,0,0,0,
	   0,0,0,10.0,0,0,
	   0,0,0,0,10.0,0,
	   0,0,0,0,0,10.0;

  double X, Y, phi;
  double v_x, v_y, r;

  while ( ros::ok() ){
    X   = z_EKF(0,0);
    Y   = z_EKF(0,1);
    phi = z_EKF(0,2);
    v_x = z_EKF(0,3);
    v_y = z_EKF(0,4);
    r   = z_EKF(0,5);

    barc_cpp::six_states six_states;
    six_states.X =   X;
    six_states.Y =   Y;
    six_states.yaw = phi;
    six_states.vx =  v_x;
    six_states.vy =  v_y;
    six_states.yr =  r;

    // state_estimate로 퍼블리시
    state_pub.publish(six_states);

    if (v_x_optic > v_x_min) {
      Eigen::MatrixXf y(1,6);
      Eigen::MatrixXf u(1,2);
      Eigen::MatrixXf F_ext(1,2);
      y <<     X_gps, Y_gps, yaw_local, v_x_optic, v_y_optic, w_z;
      u <<     d_f,      FxR;
      F_ext << a0,       Ff;

      std::vector<Eigen::MatrixXf> args;
      args.push_back(u);
      args.push_back(vhMdl);
      args.push_back(TrMdl);  // assume TrMdlFront == TrMdlRear
      args.push_back(F_ext);

      std::tie(z_EKF, P) = ekf(f_6s, z_EKF, P, h_6s, y, Q, R, args, dt);
    }
    else{
      z_EKF(0,0) = X_gps;
      z_EKF(0,1) = Y_gps;
      z_EKF(0,2) = yaw_local;
      z_EKF(0,3) = v_x_optic;
      z_EKF(0,4) = v_y_optic;
      z_EKF(0,5) = w_z;
    }
    rate.sleep();
  }

  return 0;
}
