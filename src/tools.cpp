#include "tools.h"
#include <iostream>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;
using std::cout;
using std::endl;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
   * TODO: Calculate the RMSE here.
   */
   VectorXd rmse(4);
   rmse << 0,0,0,0;

   int n = estimations.size();
   if(n==0 || n != ground_truth.size())
   {
      cout<<"Error. Size of estimation is not correct"<<endl;
      return rmse;
   }

   VectorXd res_i(4);
   for (int i=0; i < n; ++i) {
      res_i = estimations[i] - ground_truth[i];
      res_i = res_i.array()*res_i.array();
      rmse += res_i;
   }

   // TODO: calculate the mean
   rmse = rmse.array()/n;
   // TODO: calculate the squared root
   rmse = rmse.array().sqrt();
   // return the result
   return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
   * TODO:
   * Calculate a Jacobian here.
   */

   MatrixXd Hj(3,4);
   Hj<< 0, 0, 0, 0,
        0, 0, 0, 0,
        0, 0, 0, 0;
      
   // recover state parameters
   float px = x_state(0);
   float py = x_state(1);
   float vx = x_state(2);
   float vy = x_state(3);

   // check division by zero
   if(px==0 && py==0)
   {
      cout<<"Error, Division by zero"<<endl;
      return Hj;
   }
   // compute the Jacobian matrix
   float px2_py2 = px*px+py*py;
   float sqrt_px2_py2 = sqrt(px2_py2);
   float px2_py2_3_2 = px2_py2*sqrt_px2_py2;
   Hj << px/sqrt_px2_py2, py/sqrt_px2_py2, 0, 0,
         -py/px2_py2, px/px2_py2, 0, 0,
         py*(vx*py-vy*px)/px2_py2_3_2, px*(vy*px-vx*py)/px2_py2_3_2, px/sqrt_px2_py2, py/sqrt_px2_py2;
   return Hj;
}
