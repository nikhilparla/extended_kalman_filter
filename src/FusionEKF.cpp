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
	ekf_.x_ = VectorXd(4);
	
	// state covarience matrix
	ekf_.P_ = MatrixXd(4,4);
	ekf_.P_ << 	1,0,0,0,
			0,1,0,0, 
			0,0,1000,0,
			0,0,0,1000;
	
	// measurement matrix
	H_laser_ << 1,0,0,0,
				0,1,0,0;
	
	 // initial state transition matrix F
	ekf_.F_ = MatrixXd(4,4);
	ekf_.F_ << 	1,0,1,0,
			0,1,0,1,
			0,0,1,0,
			0,0,0,1;
	
	ekf_.Q_ = MatrixXd(4, 4);
		 
	ekf_.Init(ekf_.x_, ekf_.P_, ekf_.F_,H_laser_, R_laser_, ekf_.Q_);
}

/**
 * Destructor.
 */
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
  /**
   * Initialization
   */
  if (!is_initialized_) {
    /**
     * TODO: Initialize the state ekf_.x_ with the first measurement.
     * TODO: Create the covariance matrix.
     * You'll need to convert radar from polar to cartesian coordinates.
     */
	

    // first measurement
    cout << "EKF: " << endl;
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 1, 1, 1, 1;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      // TODO: Convert radar from polar to cartesian coordinates 
      //         and initialize state.
	ekf_.x_ << 	measurement_pack.raw_measurements_[0]*cos(measurement_pack.raw_measurements_[1]),
				measurement_pack.raw_measurements_[0]*sin(measurement_pack.raw_measurements_[1]),
				measurement_pack.raw_measurements_[2]*cos(measurement_pack.raw_measurements_[1]),
				measurement_pack.raw_measurements_[2]*sin(measurement_pack.raw_measurements_[1]);

    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      // TODO: Initialize state.
	ekf_.x_ << 	measurement_pack.raw_measurements_[0],
				measurement_pack.raw_measurements_[1],
				0,
				0;
    }
	previous_timestamp_ = measurement_pack.timestamp_;

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
   
   // compute the elapsed time
   float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
   previous_timestamp_ = measurement_pack.timestamp_;
   
   // 1. Modify the F matrix so that the time is integrated
   ekf_.F_ << 1, 0, dt, 0,
             0, 1, 0, dt,
	         0, 0, 1, 0,		 
	          0, 0, 0, 1;

	//set the accelation noise components
	int noise_ax = 9;
	int noise_ay = 9;
		  
			  
    float dt_pow_4 = dt*dt*dt*dt;
	float dt_pow_3 = dt*dt*dt;
	float dt_pow_2 = dt*dt;

	// set the proces covariance matrix
	ekf_.Q_ = MatrixXd(4, 4);
	ekf_.Q_ <<   dt_pow_4/4*noise_ax, 0, dt_pow_3/2*noise_ax, 0,
				0, dt_pow_4/4*noise_ay, 0, dt_pow_3/2*noise_ay,
				dt_pow_3/2*noise_ax, 0, dt_pow_2*noise_ax, 0,
				0, dt_pow_3/2*noise_ay, 0, dt_pow_2*noise_ay;
	
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
    // TODO: Radar updates
	MatrixXd Hj_ = tools.CalculateJacobian(ekf_.x_);
	ekf_.H_ = Hj_;
	ekf_.R_ = R_radar_;
	ekf_.UpdateEKF(measurement_pack.raw_measurements_);

  } else {
    // TODO: Laser updates
		ekf_.R_ = R_laser_;
		// Update H_ for laser
		H_laser_ << 1.0, 0.0, dt, 0.0,
					0.0, 1.0, 0.0, dt;
		ekf_.H_ = H_laser_;
		ekf_.Update(measurement_pack.raw_measurements_);
	}
	
  

  // print the output
	if (measurement_pack.sensor_type_ == MeasurementPackage::LASER)
		cout << "Current = " << measurement_pack.raw_measurements_[1] << endl;
	else
		cout << "Current = " << measurement_pack.raw_measurements_[0]*sin(measurement_pack.raw_measurements_[1]) << endl;
  cout << "x_ = " << ekf_.x_(0) << endl;
  //cout << "P_ = " << ekf_.P_ << endl;
}
