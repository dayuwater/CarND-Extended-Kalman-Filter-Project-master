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
    
    
    // initializing matrices
    
    // Measurement covariance
    R_laser_ = MatrixXd(2, 2);
    R_radar_ = MatrixXd(3, 3);
    
    // Measurement Matrix
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
     TODO:
     * Finish initializing the FusionEKF.
     * Set the process and measurement noises
     */
    
    //create a 4D state vector, we don't know yet the values of the x state
    ekf_.x_ = VectorXd(4);
    
    //state covariance matrix P
    ekf_.P_ = MatrixXd(4, 4);
    ekf_.P_ << 1, 0, 0, 0,
    0, 1, 0, 0,
    0, 0, 1000, 0,
    0, 0, 0, 1000;
    
    //the initial transition matrix F_
    ekf_.F_ = MatrixXd(4, 4);
    ekf_.F_ << 1, 0, 1, 0,
    0, 1, 0, 1,
    0, 0, 1, 0,
    0, 0, 0, 1;
    
    //measurement matrix
    H_laser_ << 1, 0, 0, 0,
        0, 1, 0, 0;
    
    //measurement covariance
    ekf_.R_ = MatrixXd(2,2);
    ekf_.R_ << R_laser_;
 
    
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
        /**
         TODO:
         * Initialize the state ekf_.x_ with the first measurement.
         * Create the covariance matrix. (Q, R)
         * Remember: you'll need to convert radar from polar to cartesian coordinates.
         */
        // first measurement
        cout << "EKF: " << endl;
        ekf_.x_ = VectorXd(4);
        ekf_.x_ << 1, 1, 1, 1;
        
        if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
            /**
             Convert radar from polar to cartesian coordinates and initialize state.
             */
            
            VectorXd x_polar = VectorXd(3);
            
            cout << "RM:" << measurement_pack.raw_measurements_ << endl;
            // rho, theta, rhodot
            
            float rho = measurement_pack.raw_measurements_(0);
            float theta = measurement_pack.raw_measurements_(1);
            theta = atan2(sin(theta), cos(theta));
            
            // convert polar to cartesian
            float px = rho * cos(theta);
            float py = rho * sin(theta);
           
            ekf_.x_ = VectorXd(4);
            ekf_.x_ << px, py, 0, 0;
            
            
            
            
            
            
            
        }
        else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
            /**
             Initialize state. 
             */
            ekf_.x_ = VectorXd(4);
            ekf_.x_ << measurement_pack.raw_measurements_(0), measurement_pack.raw_measurements_(1), 0, 0;

            
            
            

        }
        
        // done initializing, no need to predict or update
        previous_timestamp_ = measurement_pack.timestamp_;
        is_initialized_ = true;
        return;
    }
    
    /*****************************************************************************
     *  Prediction
     ****************************************************************************/
    
    /**
     TODO:
     * Update the state transition matrix F according to the new elapsed time.
     - Time is measured in seconds.
     * Update the process noise covariance matrix. ( Q )
     * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
     */
    
    float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
    previous_timestamp_ = measurement_pack.timestamp_;
    // cout << "dt" << dt << endl;
    float noise_ax = 9.0;
    float noise_ay = 9.0;
    
    ekf_.F_(0,2) = dt;
    ekf_.F_(1,3) = dt;
    
    
    
    ekf_.Q_ = MatrixXd(4,4);
    ekf_.Q_ << pow(dt,4) / 4.0 * noise_ax, 0 , pow(dt,3) / 2.0 * noise_ax, 0,
    0, pow(dt,4) / 4.0 * noise_ay, 0 ,pow(dt,3) / 2.0 * noise_ay,
    pow(dt,3) / 2.0 * noise_ax, 0, pow(dt,2)  * noise_ax, 0,
    0, pow(dt,3) / 2.0 * noise_ay, 0 , pow(dt,2) * noise_ay;
    
    
    
    ekf_.Predict();
    
    /*****************************************************************************
     *  Update
     ****************************************************************************/
    
    /**
     TODO:
     * Use the sensor type to perform the update step.
     * Update the state and covariance matrices. ( R )
     */
    
    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
        // Radar updates
        ekf_.R_ = R_radar_;
        Hj_ = tools.CalculateJacobian(ekf_.x_);
        ekf_.H_ = Hj_;
        ekf_.UpdateEKF(measurement_pack.raw_measurements_);
        // cout << measurement_pack.raw_measurements_;
    } else {
        // Laser updates
        ekf_.R_ = R_laser_;
        ekf_.H_ = H_laser_;
        ekf_.Update(measurement_pack.raw_measurements_);
        
    }
    
    // print the output
//    cout << "x_ = " << ekf_.x_ << endl;
//    cout << "P_ = " << ekf_.P_ << endl;
}
