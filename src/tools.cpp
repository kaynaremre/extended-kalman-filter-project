#include "tools.h"
#include <iostream>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;
using std::cout;
using std::endl;


Tools::Tools() {}

Tools::~Tools() {}

float root(float a){
    return sqrt(a);
}

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
  if(estimations.size() == 0){
      cout << "the estimation vector size should not be zero" << endl;
      return rmse;
  }
  //  * the estimation vector size should equal ground truth vector size
  if(estimations.size() != ground_truth.size()){
      cout << "the estimation vector size should equal ground truth vector size" << endl;
      return rmse;
  }
  // TODO: accumulate squared residuals
  VectorXd sum(estimations.size());
  for (int i=0; i < estimations.size(); ++i) {
    VectorXd residual = estimations[i] - ground_truth[i];

    // coefficient-wise multiplication
    residual = residual.array()*residual.array();
    rmse += residual;
  }

  // TODO: calculate the mean
    rmse = rmse/estimations.size();
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
    float px2 = px * px;
    float py2 = py * py;
    

    // TODO: YOUR CODE HERE 

    // check division by zero
    if((px2 + py2) == 0){
        return Hj;
    }
    // compute the Jacobian matrix
    Hj(0,0) = px / root( px2 + py2 );
    Hj(1,0) = -py / ( px2 + py2 );
    Hj(2,0) = py*(vx*py - vy*px) / root(pow(px2 + py2, 3));
    Hj(0,1) = py / root( px2 + py2 );
    Hj(1,1) = px / (px2 + py2);
    Hj(2,1) = px*(vy*px - vx*py) / root(pow(px2 + py2, 3));
    Hj(0,2) = 0;
    Hj(1,2) = 0;
    Hj(2,2) = px / root( px2 + py2 );
    Hj(0,3) = 0;
    Hj(1,3) = 0;
    Hj(2,3) = py / root( px2 + py2 );

    return Hj;
}
