#include "FusionEKF.h"
#include <iostream>
#include "Eigen/Dense"
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;
using std::vector;

FusionEKF::FusionEKF() {
  is_initialized_ = false;

  previous_timestamp_ = 0;

  // Initialize KF state
  x_ = VectorXd(4);

  // state covariance matrix P
  P_ = MatrixXd(4, 4);
  P_ << 1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1000, 0,
        0, 0, 0, 1000;

  // the initial transition matrix F_
  F_ = MatrixXd(4, 4);
  F_ << 1, 0, 1, 0,
        0, 1, 0, 1,
        0, 0, 1, 0,
        0, 0, 0, 1;

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

  noise_ax = 9;
  noise_ay = 9;

  Q_ = MatrixXd(4, 4);

}

FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {

  float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
  previous_timestamp_ = measurement_pack.timestamp_;

  F_(0, 2) = dt;
  F_(1, 3) = dt;

  double t1 = dt * dt * dt * dt / 4;
  double t2 = dt * dt * dt / 2;
  double t3 = dt * dt;
  double q1x = t1 * noise_ax;
  double q2x = t2 * noise_ax;
  double q3x = t3 * noise_ax;
  double q1y = t1 * noise_ay;
  double q2y = t2 * noise_ay;
  double q3y = t3 * noise_ay;

  Q_ << q1x, 0, q2x, 0,
        0, q1y, 0, q2y,
        q2x, 0, q3x, 0,
        0, q2y, 0, q3y;

  if (!is_initialized_) {

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {

      double rho = measurement_pack.raw_measurements_[0];
      double si = measurement_pack.raw_measurements_[1];
      double rho_dot = measurement_pack.raw_measurements_[2];

      x_ << rho * cos(si), 
            rho * sin(si), 
            0, // Approximate velocity assume no variation in si
            0;

      ekf_.x_ = x_;
      ekf_.P_ = P_;
      ekf_.F_ = F_;
      ekf_.Q_ = Q_;
      
    }

    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      // TODO: Initialize state.
      x_ << measurement_pack.raw_measurements_[0], 
              measurement_pack.raw_measurements_[1], 
              0, 
              0;

      ekf_.Init(x_, P_, F_, H_laser_, R_laser_, Q_);

    }

    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  ekf_.Predict();

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR && (ekf_.x_[0] != 0 || ekf_.x_[1] != 0)) {

    ekf_.H_ = tools.CalculateJacobian(ekf_.x_);
    ekf_.F_ = F_;
    ekf_.R_ = R_radar_;
    ekf_.Q_ = Q_;
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);

  } else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {

      ekf_.H_ = H_laser_;
      ekf_.F_ = F_;
      ekf_.R_ = R_laser_;
      ekf_.Q_ = Q_;
      ekf_.Update(measurement_pack.raw_measurements_);

  }
  // std::cout << "(" << ekf_.x_[0] << ", " << ekf_.x_[1] << ")" << std::endl;

}
