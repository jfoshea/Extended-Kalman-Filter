#include "kalman_filter.h"
#include "tools.h"
#include <iostream> 

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;

// Please note that the Eigen library does not initialize 
// VectorXd or MatrixXd objects with zeros upon creation.

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

//=============================================================================
//  @brief: CartersionToPolar()
//          Converts a 1x4 Cartersion vector to a 1x3 Polar vector 
//  @params: 1x4 Cartesian Vector 
//  @return: 1x3 Polar Vector 
//=============================================================================
VectorXd KalmanFilter::CartesianToPolar( const VectorXd &v_in ) {

  VectorXd v_out( 3 );
  v_out << 1, 1, 1;

  // Check input state parameters for a potential division by zero
  if( v_in(0) == 0 && v_in(1) == 0 ) {
    cout << "Error:" << __func__ << ": Division by zero detected" << endl;
    return v_out;
  }

  auto px = v_in(0);
  auto py = v_in(1);
  auto vx = v_in(2);
  auto vy = v_in(3);

  auto rho = sqrt( (px * px) + (py * py) );
  auto phi = atan2( py, px );
  auto rho_dot = ( (px * vx) + (py * vy) ) / rho;

  v_out << rho, phi, rho_dot;

  return v_out;
}

//=============================================================================
//  @brief: PolarToCartersion()
//          Converts 1x3 Polar to 1x4 Cartersion vector
//          and returning initialized 1x4 vector.
//  @params: 1x3 Polar Vector 
//  @return: 1x4 Cartesian Vector 
//=============================================================================
VectorXd KalmanFilter::PolarToCartesian( const VectorXd &v_in ) {

  VectorXd v_out( 4 );

  auto rho = v_in(0);
  auto phi = v_in(1);
  auto rho_dot = v_in(2);

  auto px = rho * cos( phi );
  auto py = rho * sin( phi );
  auto vx = rho_dot * cos( phi );
  auto vy = rho_dot * sin( phi );

  v_out << px, py, vx, vy;

  return v_out;
}

//=============================================================================
//  @brief: InitializeRadarVector()
//            Initializes a Radar Vector by converting 1x3 Polar vector to a 
//            1x4 Cartersion vector and returning initialized 1x4 vector
//  @params:  1x3 Polar Vector 
//  @return:  1x4 Cartesian Vector 
//=============================================================================
VectorXd KalmanFilter::InitializeRadarVector( const VectorXd &v_in ) {

  VectorXd v_out( 4 );
  v_out << 1, 1, 1, 1;

  // Check input state parameters for a potential division by zero
  if( v_in(0) == 0 && v_in(1) == 0 ) {
    cout << "Error:" << __func__ << ": Division by zero detected" << endl;
    return v_out;
  }

  v_out = PolarToCartesian( v_in );

  return v_out;
}

//=============================================================================
//  @brief: InitializeLidarVector()
//          Initializes a Lidar Vector which is already in Cartersion form 
//  @params: 1x4 Catesian Vector 
//  @return: 1x4 Initialized Lidar Vector 
//=============================================================================
VectorXd KalmanFilter::InitializeLidarVector( const VectorXd &v_in ) {

  VectorXd v_out( 4 );

  v_out << v_in( 0 ),v_in( 1 ), 0, 0;

  return v_out;
}

//=============================================================================
//  @brief: SetStateTransitionMatrix():
//            Sets/Updates State Transition Matrix F with Delta T  
//  @params:  const float dt
//  @return:  Updated State Transition Matrix F 
//=============================================================================
MatrixXd KalmanFilter::SetStateTransitionMatrix( const float dt ) {

	MatrixXd F = MatrixXd( 4, 4 );

  F <<  1, 0, dt, 0,
        0, 1, 0, dt,
        0, 0, 1, 0,
        0, 0, 0, 1;

  return F;
}

//=============================================================================
//  @brief: SetCovarianceMatrix():
//          Sets/Updates the Covariance Matrix with Delta T and noise  
//  @params:  const float dt
//            const float noise_ax
//            const float noise_ay
//  @return: Updated Covariance Matrix Q 
//=============================================================================
MatrixXd KalmanFilter::SetCovarianceMatrix( const float dt, const float noise_ax, const float noise_ay ){


  auto dt2 = dt * dt;
  auto dt3 = dt2 * dt;
  auto dt4 = dt3 * dt;

  auto r0c0  = dt4/4 * noise_ax;
  auto r0c1  = 0.0;
  auto r0c2  = dt3/2 * noise_ax;
  auto r0c3  = 0.0;

  auto r1c0  = 0.0;
  auto r1c1  = dt4/4 * noise_ay;
  auto r1c2  = 0.0;
  auto r1c3  = dt3/2 * noise_ay;

  auto r2c0  = dt3/2 * noise_ax;
  auto r2c1  = 0.0;
  auto r2c2  = dt2/2 * noise_ay;
  auto r2c3  = 0.0;

  auto r3c0  = 0.0;
  auto r3c1  = dt3/2 * noise_ay;
  auto r3c2  = 0.0;
  auto r3c3  = dt2 * noise_ay;

  MatrixXd Q = MatrixXd( 4, 4 );

  Q <<  r0c0, r0c1, r0c2, r0c3,
        r1c0, r1c1, r1c2, r1c3,
        r2c0, r2c1, r2c2, r2c3,
        r3c0, r3c1, r3c2, r3c3;

  return Q;
}

//=============================================================================
//  @brief:   Predict(): predicts next step
//  @param:   const VectorXd z
//  @return:  void 
//=============================================================================
void KalmanFilter::Predict() {

  x_ = F_ * x_;
  P_ = (F_ * P_ * F_.transpose()) + Q_;
}

//=============================================================================
//  @brief:   Update(): Updates P_ state covariance matrix for Lidar
//  @param:   const VectorXd z
//  @return:  void 
//=============================================================================
void KalmanFilter::Update( const VectorXd &z ) {

  VectorXd y = z - H_ * x_;
  MatrixXd S = H_ * P_ * H_.transpose() + R_;
  MatrixXd K = P_ * H_.transpose() * S.inverse();

  // New Estimates
  x_ = x_ + ( K * y );
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity( x_size, x_size );
  P_ = ( I - K * H_ ) * P_;
}

//=============================================================================
//  @brief:   UpdateEKF(): Updates P_ state covariance matrix for Radar
//  @param:   const VectorXd z
//  @return:  void 
//=============================================================================
void KalmanFilter::UpdateEKF(const VectorXd &z) {

  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);

  VectorXd y = z - CartesianToPolar( x_ );
  MatrixXd S = H_ * P_ * H_.transpose() + R_;
  MatrixXd K = P_ * H_.transpose() * S.inverse();

  while ( y(1) > M_PI || y(1) < -M_PI ) {
    if ( y(1) > M_PI ) {
      y(1) -= M_PI;
    } else {
      y(1) += M_PI;
    }
  }

  // New Estimates
  x_ = x_ + K * y;
  P_ = ( I - K * H_ ) * P_;
}
