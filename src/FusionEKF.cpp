#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/*
 * Constructor.
 */
FusionEKF::FusionEKF() {
  is_initialized_ = false;

  previous_timestamp_ = 0;

  // initializing matrices
  R_laser_ = MatrixXd( 2, 2 );
  R_radar_ = MatrixXd( 3, 3 );
  H_laser_ = MatrixXd( 2, 4 );
  Hj_ = MatrixXd( 3, 4 );
  P_radar_ = MatrixXd( 4, 4 );
  P_laser_ = MatrixXd( 4, 4 );

  // measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
              0, 0.0225;

  // measurement covariance matrix - radar
  R_radar_ << 0.09, 0,      0,
              0,    0.0009, 0,
              0,    0,      0.09;

  P_radar_ << 1, 0, 0, 0,
              0, 1, 0, 0,
              0, 0, 1, 0,
              0, 0, 0, 1;

  P_laser_ << 1, 0, 0, 0,
              0, 1, 0, 0,
              0, 0, 1000, 0,
              0, 0, 0, 1000;

  H_laser_ << 1, 0, 0, 0,
              0, 1, 0, 0;

}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {
    cout << "EKF: " << endl;
    ekf_.x_ = VectorXd( 4 );
    ekf_.x_ << 1, 1, 1, 1;

    if ( measurement_pack.sensor_type_ == MeasurementPackage::RADAR ) {
      // Initialize radar vector 
      VectorXd x_in = ekf_.InitializeRadarVector( measurement_pack.raw_measurements_ );

      // Initialize P_in for Lidar/Laser
      MatrixXd P_in = P_radar_;

      // Initialize the state transition matrix F_in
      MatrixXd F_in = ekf_.SetStateTransitionMatrix( 0 );

      // Initialize Jacobian Matrix using x_in
      MatrixXd H_in = tools.CalculateJacobian( x_in );

      // Initialize R_in with initial radar values
      MatrixXd R_in = R_radar_;

      // Set initial process covariance using given noise and dt = 0
      MatrixXd Q_in = ekf_.SetCovarianceMatrix( 0, noise_ax, noise_ay );

      ekf_.Init( x_in, P_in, F_in, H_in, R_in, Q_in );
    }
    else if ( measurement_pack.sensor_type_ == MeasurementPackage::LASER ) {
      // Initialize lidar vector 
      VectorXd x_in = ekf_.InitializeLidarVector( measurement_pack.raw_measurements_ );

      // Initialize P_in for Lidar/Laser
      MatrixXd P_in = P_laser_;

      // Initialize the state transition matrix F_in
      MatrixXd F_in = ekf_.SetStateTransitionMatrix( 0 );

      // Initialize Jacobian Matrix using x_in
      MatrixXd H_in = tools.CalculateJacobian( x_in );

      // Initialize R_in with initial laser values
      MatrixXd R_in = R_laser_;

      // Initialize process covariance with process noise and dt = 0
      MatrixXd Q_in = ekf_.SetCovarianceMatrix( 0, noise_ax, noise_ay );

      ekf_.Init( x_in, P_in, F_in, H_in, R_in, Q_in );
    }

    // Saving first timestamp in seconds
    previous_timestamp_ = measurement_pack.timestamp_ ;

    is_initialized_ = true;

    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/
  auto dt = ( measurement_pack.timestamp_ - previous_timestamp_ ) / 1000000.0;

  previous_timestamp_ = measurement_pack.timestamp_;

  // Update the state transition matrix F to new elapsed time
  ekf_.F_ = ekf_.SetStateTransitionMatrix( dt );

  // Update the process noise covariance matrix
  ekf_.Q_ = ekf_.SetCovarianceMatrix( dt, noise_ax, noise_ay );

  ekf_.Predict();

  /*****************************************************************************
   *  Update
   ****************************************************************************/

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates
    ekf_.H_ = tools.CalculateJacobian( ekf_.x_ );
    ekf_.R_ = R_radar_;
    ekf_.UpdateEKF( measurement_pack.raw_measurements_ );
  } else {
    // Laser updates
    ekf_.H_ = H_laser_;
    ekf_.R_ = R_laser_;
    ekf_.Update( measurement_pack.raw_measurements_ );
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;

}
