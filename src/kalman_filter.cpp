#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

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
  x_ = F_ * x_;
  P_ = F_ * P_ * F_.transpose() + Q_;
}


// for lidar
void KalmanFilter::Update(const VectorXd &z) {
  VectorXd z_pred = H_ * x_;
	VectorXd y = z - z_pred;
	
	MatrixXd Ht = H_.transpose();

	MatrixXd S = H_ * P_ * Ht + R_;
	MatrixXd Si = S.inverse();
	MatrixXd PHt = P_ * Ht;
	MatrixXd K = PHt * Si;
	//new estimate
	x_ = x_ + (K * y);
	long x_size = x_.size();
	MatrixXd I = MatrixXd::Identity(x_size, x_size);
	P_ = (I - K * H_) * P_;

}

// for radar
void KalmanFilter::UpdateEKF(const VectorXd &z) {
  float px = x_[0];
	float py = x_[1];
	float vx = x_[2];
	float vy = x_[3];

  VectorXd z_pred(3);
  double pi = 3.14159265359;
  float ro = sqrt(px*px + py*py);
  float phi = atan2(py,px);
  
  z_pred << ro, phi, (px*vx + py*vy) / ro; 
	VectorXd y = z - z_pred;
	
  phi = y[1];

  while(phi > pi || phi < -pi){
    if(phi > pi){
      phi -= 2*pi;
    }else if(phi < -pi){
      phi += 2*pi;
    }
  }

  y[1] = phi;


	MatrixXd Ht = H_.transpose();
	MatrixXd S = H_ * P_ * Ht + R_;
	MatrixXd Si = S.inverse();
	MatrixXd PHt = P_ * Ht;
	MatrixXd K = PHt * Si;
	//new estimate
	x_ = x_ + (K * y);
	long x_size = x_.size();
	MatrixXd I = MatrixXd::Identity(x_size, x_size);
	P_ = (I - K * H_) * P_;
}
