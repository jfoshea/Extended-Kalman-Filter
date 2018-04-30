#ifndef KALMAN_FILTER_H_
#define KALMAN_FILTER_H_
#include "Eigen/Dense"

class KalmanFilter {
private:
  /**
  *   Common calculation for KF and EKF.
  *   @param y.
  */
  void UpdateWithY(const Eigen::VectorXd &y);
public:

  // state vector
  Eigen::VectorXd x_;

  // state covariance matrix
  Eigen::MatrixXd P_;

  // state transition matrix
  Eigen::MatrixXd F_;

  // process covariance matrix
  Eigen::MatrixXd Q_;

  // measurement matrix
  Eigen::MatrixXd H_;

  // measurement covariance matrix
  Eigen::MatrixXd R_;

  /**
   * Constructor
   */
  KalmanFilter();

  /**
   * Destructor
   */
  virtual ~KalmanFilter();

  /**
   * Init Initializes Kalman filter
   * @param x_in Initial state
   * @param P_in Initial state covariance
   * @param F_in Transition matrix
   * @param H_in Measurement matrix
   * @param R_in Measurement covariance matrix
   * @param Q_in Process covariance matrix
   */
  void Init(Eigen::VectorXd &x_in, Eigen::MatrixXd &P_in, Eigen::MatrixXd &F_in,
      Eigen::MatrixXd &H_in, Eigen::MatrixXd &R_in, Eigen::MatrixXd &Q_in);

  /**
   * Prediction Predicts the state and the state covariance
   * using the process model
   * @param delta_T Time between k and k+1 in s
   */
  void Predict();

  /**
  * A helper method to convert Polar to Cartesian vector.
  */
  Eigen::VectorXd PolarToCartesian( const Eigen::VectorXd &v_in );

  /**
  * A helper method to convert Cartesian to Polar vector.
  */
  Eigen::VectorXd CartesianToPolar( const Eigen::VectorXd &v_in );

  /**
  * A helper method to initialize a Radar vector in cartesian form.
  */
  Eigen::VectorXd InitializeRadarVector( const Eigen::VectorXd &v_in );

  /**
  * A helper method to initialize a Lidar vector in cartesian form.
  */
  Eigen::VectorXd InitializeLidarVector( const Eigen::VectorXd &v_in );

  /**
  * A helper method to set/update the transistion matrix.
  */
  Eigen::MatrixXd SetStateTransitionMatrix( const float dt );

  /**
  * A helper method to set/update the covariance matrix.
  */
  Eigen::MatrixXd SetCovarianceMatrix( const float dt, const float noise_ax, const float noise_ay );

  /**
   * Updates the state by using standard Kalman Filter equations
   * @param z The measurement at k+1
   */
  void Update(const Eigen::VectorXd &z);

  /**
   * Updates the state by using Extended Kalman Filter equations
   * @param z The measurement at k+1
   */
  void UpdateEKF(const Eigen::VectorXd &z);

};

#endif /* KALMAN_FILTER_H_ */
