#include "FusionEKF.h"
#include <iostream>
#include "Eigen/Dense"
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;
using std::vector;

/**
 * Constructor.
 */
FusionEKF::FusionEKF() {
  is_initialized_ = false;

  previous_timestamp_ = 0;

  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  
  H_laser_ = MatrixXd(2, 4);
  H_laser_ << 1, 0, 0, 0,
            0, 1, 0, 0;

  Hj_ = MatrixXd(3, 4);

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
              0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
              0, 0.0009, 0,
              0, 0, 0.09;
  /**
   * TODO: Finish initializing the FusionEKF.
   * TODO: Set the process and measurement noises
   */

  // the initial transition matrix F_
  MatrixXd F_ = MatrixXd(4, 4);
  F_ << 1, 0, 1, 0,
            0, 1, 0, 1,
            0, 0, 1, 0,
            0, 0, 0, 1;

  MatrixXd P_ = MatrixXd(4, 4);
  P_ << 1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1000, 0,
        0, 0, 0, 1000;

}

/**
 * Destructor.
 */
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {

  float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
  previous_timestamp_ = measurement_pack.timestamp_;

  double noise_ax = 9;
  double noise_ay = 9;

  /**
   * Initialization
   */
  if (!is_initialized_) {
    VectorXd x_ = VectorXd(4);
    /**
     * TODO: Initialize the state ekf_.x_ with the first measurement.
     * TODO: Create the covariance matrix.
     * You'll need to convert radar from polar to cartesian coordinates.
     */
    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      // TODO: Convert radar from polar to cartesian coordinates 
      //         and initialize state.

      double rho = measurement_pack.raw_measurements_[0];
      double si = measurement_pack.raw_measurements_[1];
      double rho_dot = measurement_pack.raw_measurements_[2];

      x_ << rho * cos(si), 
              rho * sin(si), 
              rho_dot * cos(si), 
              rho_dot * sin(si);

      Hj_ = tools.CalculateJacobian(ekf_.x_);

    }

    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      // TODO: Initialize state.
      ekf_.x_ << measurement_pack.raw_measurements_[0], 
              measurement_pack.raw_measurements_[1], 
              0, 
              0;
      ekf_.H_ = H_laser_;
      ekf_.R_ = R__;
    }

    ekf_.Q_ = MatrixXd(4, 4);
  
    double t1 = dt * dt * dt * dt / 4;
    double t2 = dt * dt * dt / 2;
    double t3 = dt * dt;
    double q1x = t1 * noise_ax;
    double q2x = t2 * noise_ax;
    double q3x = t3 * noise_ax;
    double q1y = t1 * noise_ay;
    double q2y = t2 * noise_ay;
    double q3y = t3 * noise_ay;
    
    ekf_.Q_ << q1x, 0, q2x, 0,
              0, q1y, 0, q2y,
              q2x, 0, q3x, 0,
              0, q2y, 0, q3y;

    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /**
   * Prediction
   */

  /**
   * TODO: Update the state transition matrix F according to the new elapsed time.
   * Time is measured in seconds.
   * TODO: Update the process noise covariance matrix.
   * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */

  ekf_.F_(0, 2) = dt;
  ekf_.F_(1, 3) = dt;

  ekf_.Predict();

  /**
   * Update
   */

  /**
   * TODO:
   * - Use the sensor type to perform the update step.
   * - Update the state and covariance matrices.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);

  } else {
    ekf_.Update(measurement_pack.raw_measurements_);
  }

}
