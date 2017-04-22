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
  if(estimations.size() == 0 || (estimations.size() != ground_truth.size()) ){
    std::cout << "Error with input into CalculateRMSE function" << std::endl;
    return rmse;
	}

	//accumulate squared residuals
	for(unsigned int i=0; i < estimations.size(); ++i){
		VectorXd c = estimations[i] - ground_truth[i];
		c = c.array() * c.array();
		rmse += c;
	}

  rmse = rmse / estimations.size();
  rmse = rmse.array().sqrt();
  return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  MatrixXd Hj = MatrixXd::Zero(3,4);
	//recover state parameters
	float px = x_state(0);
	float py = x_state(1);
	float vx = x_state(2);
	float vy = x_state(3);

  float c = (px*px + py*py);
  float c2 = sqrt(c);
  float c3 = c * c2;

  if(fabs(c) < 0.0001){
      std::cout << "Error, px and py == 0 cannot calculate Jacobian" << std::endl;
      return Hj;
  }
  Hj(0,0) = px / c2;
  Hj(0,1) = py / c2;
  Hj(1,0) = -(py / c);
  Hj(1,1) = (px / c);
  
  Hj(2,0) = py * (vx*py - px*vy) / c3;
  Hj(2,1) = px * (vy*px - py*vx) / c3;
  Hj(2,2) = px / c2;
  Hj(2,3) = py / c2;
	
	return Hj;
}
