#include <Eigen/Core>          // for matrix
#include <Eigen/Dense>         // for matrix
#include <tuple>               // for multiple values return

#include <functional>
#include <vector>
#include <algorithm>

using namespace std;

//Eigen::Array3f (*f_3s_ptr) (Eigen::Array3f, std::vector<Eigen::ArrayXf>); //def f_3s(z, u, vhMdl, trMdl, F_ext, dt):

Eigen::MatrixXf numerical_jac(
    std::function<Eigen::MatrixXf(Eigen::MatrixXf, std::vector<Eigen::MatrixXf>, double)> f,
    Eigen::MatrixXf x,
    std::vector<Eigen::MatrixXf> args,
    double dt)
{
  Eigen::MatrixXf y = f(x, args, dt);  // return [1,3] matrix
  Eigen::MatrixXf xp(x);
  double eps = 1e-5;

  // create zero matrix
  Eigen::MatrixXf jac(y.cols(),x.cols());
  jac.setZero(y.cols(), x.cols());

  Eigen::MatrixXf yhi, ylo;

  for(int i =0; i < x.cols(); i++){
    xp(0,i) = x(0,i) + eps/2.0;
    yhi = f(xp, args, dt);
    xp(0,i) = x(0,i) - eps/2.0;
    ylo = f(xp, args, dt);
    xp(0,i) = x(0,i);
    jac.col(i) = (yhi - ylo) / eps;
  }

  return jac;
}

Eigen::MatrixXf numerical_jac2(
    std::function<Eigen::MatrixXf(Eigen::MatrixXf)> h,
    Eigen::MatrixXf x)
{
  Eigen::MatrixXf y = h(x);  // return [1, 2] matrix
  Eigen::MatrixXf xp(x);
  double eps = 1e-5;

  // create zero matrix
  Eigen::MatrixXf jac(y.cols(), x.cols());  // [2x3]
  jac.setZero(y.cols(), x.cols());

  Eigen::MatrixXf yhi, ylo;

  for(int i =0; i < x.cols(); i++){
    xp(0,i) = x(0,i) + eps/2.0;
    yhi = h(xp);
    xp(0,i) = x(0,i) - eps/2.0;
    ylo = h(xp);
    xp(0,i) = x(0,i);
    jac.col(i) = (yhi - ylo) / eps;
  }

  return jac;  // [2x3]
}



std::tuple<Eigen::MatrixXf, Eigen::MatrixXf> ekf (
    std::function<Eigen::MatrixXf(Eigen::MatrixXf, std::vector<Eigen::MatrixXf>, double)> f,
    Eigen::MatrixXf mx_k,  // z_EKF
    Eigen::MatrixXf P_k,
    std::function<Eigen::MatrixXf(Eigen::MatrixXf)> h,
    Eigen::MatrixXf y_kp1,
    Eigen::MatrixXf Q,
    Eigen::MatrixXf R,
    std::vector<Eigen::MatrixXf> args,
    double dt)
{
  double xDim = mx_k.cols();
  Eigen::MatrixXf mx_kp1 = f(mx_k, args, dt);              // [1x3] matrix
  Eigen::MatrixXf A = numerical_jac(f, mx_k, args, dt);    // [3x3]
  Eigen::MatrixXf P_kp1 = (A * P_k) * A.transpose() + Q;   // [3x3]
  Eigen::MatrixXf my_kp1 = h(mx_kp1);                      // [1x2]
  Eigen::MatrixXf H = numerical_jac2(h, mx_kp1);           // [2x3]
  Eigen::MatrixXf P12 = P_kp1 * H.transpose();             // [3x2]
  Eigen::MatrixXf K = P12 * (((H*P12) + R).inverse());     // [3x2]
  mx_kp1 = mx_kp1 + K * (y_kp1 - my_kp1);                  // [1x3]

  // create diagonal matrix 3x3
  Eigen::Matrix3f eye3;
  eye3 << 1,0,0,
          0,1,0,
          0,0,1;

  P_kp1 = (K*R)*K.transpose() + ((eye3 - (K*H)) * P_kp1) * (eye3 - (K*H).transpose());  // [3x3]

  return std::make_tuple(mx_kp1, P_kp1);  // [1x3] , [3x3]
}


