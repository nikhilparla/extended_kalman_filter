#include "kalman_filter.h"
#include "tools.h"
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

using namespace std;

/* 
 * Please note that the Eigen library does not initialize 
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */

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

void KalmanFilter::Predict() {
  /**
   * TODO: predict the state
   */
   x_ = F_ * x_;
   P_ = F_ * P_ * F_.transpose() + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
   * TODO: update the state by using Kalman Filter equations
   */
   VectorXd z_pred = H_ * x_;
   VectorXd y = z - z_pred;
   MatrixXd Ht = H_.transpose();
/*
   cout << "H --  lidar  size = "<< H_.size() << endl;
   cout << "P_ -- lidar size = "<< P_.size() << endl;
   cout << "R  -- lidar size = "<< R_.size() << endl;
*/
   MatrixXd S = H_ * P_ * Ht + R_;
   MatrixXd Si = S.inverse();
   MatrixXd PHt = P_ * Ht;
   MatrixXd K = PHt * Si;
   
   // new estimates
   x_ = x_ + (K*y);
   MatrixXd I = MatrixXd::Identity(x_.size(), x_.size());
   P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
   * TODO: update the state by using Extended Kalman Filter equations
   */
   Tools tools;
   MatrixXd Hj = tools.CalculateJacobian(x_);
   
   float c1 = sqrt(x_(0)*x_(0) + x_(1)*x_(1));
   float c2 = atan2(x_(1),x_(0));
   float c3 = x_(0)*x_(2) + x_(1)*x_(3);
   
   Eigen::VectorXd hx;
   hx = VectorXd(3);
   hx <<	c1,
			c2,
			c3/c1;
   
   VectorXd y = z - hx;
   if((y(1) < -3.14) || (y(1) > 3.14) )
		cout << "Phi value in rad = " << y(1) << endl;
   MatrixXd Hjt = Hj.transpose();
/*   cout << "Hj size = "<< Hj.size() << endl;
   cout << "P_ size = "<< P_.size() << endl;
   cout << "R size = "<< R_.size() << endl;
*/   MatrixXd S = Hj * P_ * Hjt + R_;
   MatrixXd Si = S.inverse();
   MatrixXd PHjt = P_ * Hjt;
   MatrixXd K = PHjt * Si;
   
   // new estimates
   x_ = x_ + (K*y);
   MatrixXd I = MatrixXd::Identity(x_.size(), x_.size());
   P_ = (I - K * Hj) * P_;
}











