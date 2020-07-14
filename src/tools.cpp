#include "tools.h"
#include <iostream>

using Eigen::VectorXd;
using Eigen::MatrixXd;
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

  // TODO: YOUR CODE HERE
  // check the validity of the following inputs:
  //  * the estimation vector size should not be zero
  //  * the estimation vector size should equal ground truth vector size
    
  // TODO: accumulate squared residuals

  for (int i=0; i < estimations.size(); ++i) {
    // ... your code here
    VectorXd residual = estimations[i] - ground_truth[i];
    rmse = rmse.array() + residual.array() * residual.array();
  }

  // TODO: calculate the mean
  
  rmse = rmse.array() / estimations.size();
  
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
  // recover state parameters
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);

  // TODO: YOUR CODE HERE 

  // check division by zero
  
  double den = px * px + py * py;
  double den_root = sqrt(den);
  double den_3by2 = den *den_root; 
  
  if(den == 0) {
    std::cout << "Divide by zero" << std::endl;
    return Hj;
  }
  
  // compute the Jacobian matrix
  
  Hj << px / den_root, py / den_root, 0, 0,
        -py / den, px / den, 0, 0,
        py * (vx * py - vy * px) / den_3by2, px * (vy * px - vx * py) / den_3by2, px / den_root, py / den_root;

  return Hj;   
}
