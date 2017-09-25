#include "kalman_filter.h"
#include "tools.h"
#include <iostream>

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
  /**
  TODO:
    * predict the state
  */
    // cout << "x Initial Predict:" << x_ << endl;
    x_ = F_ * x_;
    MatrixXd Ft = F_.transpose();
    P_ = F_ * P_ * Ft + Q_;
    // cout << "x Final Predict:" << x_ << endl;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
    // cout << "x Initial KF:" << x_ << endl;
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
    // cout << "x Final KF:" << x_ << endl;

}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
    * Replace H with Jacobian
    * Use Polar for intermediate calculation, but the result is in cartesian
    * h(x) transforms from cartesian to polar, Hj "transforms" back by Jacobian
  */
   
    // z is in polar, z_pred (x_) is in cartetian, we need to convert z_pred to polar

   //  cout << "x Initial EKF:" << x_ << endl;

    // extract each part of x_
    float px = x_(0);
    float py = x_(1);
    float vx = x_(2);
    float vy = x_(3);

    // convert each part to polar
    float rho = sqrt(px * px + py * py);
    float theta = atan2(py, px); // between -pi and pi
    float rhodot = (px * vx + py * vy) / sqrt(px * px + py * py);
    
//    // convert z to cartetian, delete this after it is working, debug use only
//    float zx = z(0) * cos(z(1));
//    float zy = z(0) * sin(z(1));
//    float zvx = z(2) * cos(z(1));
//    float zvy = z(2) * sin(z(1));
    
//    if(fabs(zx) < 5){
//        x_(0) = zx;
//        x_(1) = zy;
//        x_(2) = zvx;
//        x_(3) = zvy;
//        return;
//        
//    }
    
    
    if(fabs(px * px + py * py) < 0.1)
        cout << "SQRT: " << sqrt(px * px + py * py) << endl;
    VectorXd z_pred = VectorXd(3);
    z_pred << rho, theta, rhodot;
    
    // normalize the angle in measurement
    VectorXd nz = VectorXd(3);
    float normalizedAngle = atan2(sin(z(1) ) , cos(z(1) ));
    nz << z(0), normalizedAngle, z(2);
    
    VectorXd y = nz - z_pred;
    
//    if(fabs(zx) < 5){
//        y = nz - nz;
//        
//    }

    // cout << "z:" << z << endl;
//    cout << "z_pred:" << z_pred << endl;
    
    // make sure the angle is between -pi and pi
//    if(y[1] > M_PI){
//        y[1] -= 2 * M_PI;
//    }
//    else if(y[1] < -M_PI){
//        y[1] += 2 * M_PI;
//    }
    y(1) = atan2(sin(y(1)), cos(y(1)));
    
    MatrixXd Ht = H_.transpose();
//    cout << "Hj = " << Hj<< endl;
//    cout << "p_ = " << P_ << endl;
//    cout << "Ht = " << Ht << endl;
//    cout << "R_ = " << R_ << endl;

    MatrixXd S = H_ * P_ * Ht + R_;
    MatrixXd Si = S.inverse();
    MatrixXd PHt = P_ * Ht;
    MatrixXd K = PHt * Si;
    
    //new estimate. This is in cartesian
    // cout << "K:" << K << endl;
    x_ = x_ + (K * y);
    long x_size = x_.size();
    MatrixXd I = MatrixXd::Identity(x_size, x_size);
    P_ = (I - K * H_) * P_;
    // cout << "X Final EKF:" << x_ << endl;
    
//    if(fabs(zx) < 2){
//        cout << "Z: " << z(1) << endl;
//        cout << "Prediction: " << theta << endl;
//        cout << "new Theta:" << atan2(x_(1) , x_(0));
//        cout << endl;
//    }

    // cout << endl;

}
