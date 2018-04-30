#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  VectorXd rmse(4);
  rmse << 0,0,0,0;

  // check the validity of the following inputs:
  //  - the estimation vector size should not be zero
  //  - the estimation vector size should equal ground truth vector size

  if( estimations.size() != ground_truth.size() || estimations.size() == 0) {
    cout << "Invalid estimation or ground_truth data" << endl;
    return rmse;
  }

  // Accumulate squared residuals
  for( unsigned int i=0; i < estimations.size(); ++i ) {
    VectorXd residual = estimations[i] - ground_truth[i];
    residual = residual.array()*residual.array();
    rmse += residual;
  }

  // Calculate the mean
  rmse = rmse / estimations.size();

  //calculate the squared root
  rmse = rmse.array().sqrt();

  return rmse;
}

MatrixXd Tools::CalculateJacobian( const VectorXd& x_state ) {
  MatrixXd Hj( 3,4 );

  // Check input state parameters for a potential division by zero
  if( x_state(0) == 0 && x_state(1) == 0 ) {
    cout << "Error:" << __func__ << ": Division by zero detected" << endl;
    return Hj;
  }

  // Get state parameters
  auto px = x_state(0);
  auto py = x_state(1);
  auto vx = x_state(2);
  auto vy = x_state(3);

  // Pre-compute a set of terms to avoid repeated calculation
  auto c1 = ( px * px ) + ( py * py );
  auto c2 = sqrt( c1 );
  auto c3 = ( c1 * c2 );

  // Compute the Jacobian matrix
  auto r0c0 = px/c2;
  auto r0c1 = py/c2;
  auto r0c2 = 0.0;
  auto r0c3 = 0.0;
  
  auto r1c0 = -( py/c1 );
  auto r1c1 = px/c1;
  auto r1c2 = 0.0;
  auto r1c3 = 0.0;

  auto r2c0 = py *( vx*py - vy*px )/c3;
  auto r2c1 = px *( px*vy - py*vx )/c3;
  auto r2c2 = px/c2;
  auto r2c3 = py/c2;

  Hj << r0c0, r0c1, r0c2, r0c3,
        r1c0, r1c1, r1c2, r1c3,
        r2c0, r2c1, r2c2, r2c3;

  return Hj;
}
