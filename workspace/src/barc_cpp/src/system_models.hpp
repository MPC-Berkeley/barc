#include <Eigen/Core>          // for matrix
#include <Eigen/Dense>         // for matrix

#include <cmath>

#include <vector>
#include <algorithm>

using namespace std;

double f_pajecka(Eigen::Array3f trMdl, double alpha);


// discrete non-linear bicycle model dynamics
//def f_3s(z, u, vhMdl, trMdl, F_ext, dt):
Eigen::MatrixXf f_3s (Eigen::MatrixXf z,
                     std::vector<Eigen::MatrixXf> args,
                     double dt)
{
  double v_x = z(0,0);
  double v_y = z(0,1);
  double r =   z(0,2);

  /*
    args(0) = u
    args(1) = vhMdl
    args(2) = trMdl
    args(3) = F_ext
  */

  // u
  double d_f = (args.at(0))(0,0);
  double FxR = (args.at(0))(0,1);

  // vhMdl
  double a = (args.at(1))(0,0);
  double b = (args.at(1))(0,1);
  double m = (args.at(1))(0,2);
  double I_z = (args.at(1))(0,3);

  // trMdl ( assume trMdlFront == trMdlRear)
  double B = (args.at(2))(0,0);
  double C = (args.at(2))(0,1);
  double mu = (args.at(2))(0,2);

  // F_ext
  double a0 = (args.at(3))(0,0);
  double Ff = (args.at(3))(0,1);

  double g = 9.81;
  double Fn = m*g / 2.0;

  if (FxR >= mu*Fn)  FxR = mu*Fn;

  double a_F = atan((v_y + a*r) / v_x) - d_f;
  double a_R = atan((v_y - b*r) / v_x);

  Eigen::Array3f TM_param(B, C, mu*Fn);
  double FyF = -f_pajecka(TM_param, a_F);

  double FyR_paj = -f_pajecka(TM_param, a_R);
  double FyR_max = sqrt(pow(mu*Fn, 2) - pow(FxR, 2));
  double FyR = min(FyR_max, max(-FyR_max, FyR_paj));

  double v_x_next = v_x + dt*(r*v_y + 1/m * (FxR - FyF*sin(d_f)) - a0*pow(v_x,2) - Ff);
  double v_y_next = v_y + dt*(-r*v_x +1/m * (FyF*cos(d_f) + FyR));
  double r_next   = r + dt/I_z*(a*FyF*cos(d_f) - b*FyR);

  Eigen::MatrixXf retn_val(1, 3);
  retn_val(0,0) = v_x_next;
  retn_val(0,1) = v_y_next;
  retn_val(0,2) = r_next;

  return retn_val;
}


//def h_3s(x)
Eigen::MatrixXf h_3s (Eigen::MatrixXf x)
{
  /*
    x : shape(1,3)
    C : shape(3,2)
    return : shape(1,2)

    python 코드와 약간 다르다. 파이썬은 C : shape(2,3)
  */

  Eigen::MatrixXf C(3,2);
  C.setZero(3,2);
  C(0,0) = 1;
  C(2,1) = 1;

  return x * C;  // shape(1,2)
}

double f_pajecka(Eigen::Array3f trMdl,
                 double alpha)
{
  /*
    trMdl(0) = b
    trMdl(1) = c
    trMdl(2) = d
  */

  return trMdl(2) * sin( trMdl(1) * atan(trMdl(0) * alpha));
}
