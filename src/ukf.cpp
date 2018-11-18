#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 * This is scaffolding, do not modify
 */
UKF::UKF() {
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd(5);

  // initial covariance matrix
  P_ = MatrixXd(5, 5);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 30;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 30;
  
  //DO NOT MODIFY measurement noise values below these are provided by the sensor manufacturer.
  // Laser measurement noise standard deviation position1 in m
  std_laspx_ = 0.15;

  // Laser measurement noise standard deviation position2 in m
  std_laspy_ = 0.15;

  // Radar measurement noise standard deviation radius in m
  std_radr_ = 0.3;

  // Radar measurement noise standard deviation angle in rad
  std_radphi_ = 0.03;

  // Radar measurement noise standard deviation radius change in m/s
  std_radrd_ = 0.3;
  //DO NOT MODIFY measurement noise values above these are provided by the sensor manufacturer.
  
  /**
  TODO:

  Complete the initialization. See ukf.h for other member properties.

  Hint: one or more values initialized above might be wildly off...
  */

  // initially set to false, set to true in first call of Process Managerment
  is_initialized_ = false;

  // Time when state i s true, in us
  n_x_ = 5;

  ///* Augmented state dimension
  int n_aug_ = 7;

  ///* Sigma point spreading parameter
  double lambda_ = 3 - n_x_;
  
  // predicted sigma points matrix
  Xsig_pred_ = MatrixXd(n_x_, 2 * n_aug_ + 1);

  //create vector for weights
  weights_ = VectorXd(2 * n_aug_ + 1);

  // the current NIS for radar
  NIS_radar_ = 0.0;

  // the current NIS for laser
  NIS_laser_ = 0.0;
}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Make sure you switch between lidar and radar
  measurements.
  */
  

  // skip predict/update if sensor type is ignored

  if((meas_package.sensor_type_ == MeasurementPackage::RADAR && use_radar_)||
      (meas_package.sensor_type_ == MeasurementPackage::LASER && use_laser_)){

        /*****************************************************************************
        *  Initialization
        ****************************************************************************/
       if(!is_initialized_){
         // first measurement
         x_ << 1 ,1, 1, 1, 0.1;
         // init covariance matrix
         P_ << 0.15,  0, 0, 0, 0,
                0, 0.15, 0, 0, 0,
                0,    0, 1, 0, 0,
                0,    0, 0, 1, 0,
                0,    0, 0, 0, 1;

        // iniy timestamp
        time_us_ = meas_package.timestamp_;

        if(meas_package.sensor_type_ == MeasurementPackage::LASER && use_laser_){
          x_(0) = meas_package.raw_measurements_(0);
          x_(1) = meas_package.raw_measurements_(1);
        }

        else if(meas_package.sensor_type_ == MeasurementPackage::RADAR && use_radar_){
            /**
            Convert radar from polar to cartesian coordinates and initialize state.
            */
            float ro = meas_package.raw_measurements_(0);
            float phi = meas_package.raw_measurements_(1);
            float ro_dot = meas_package.raw_measurements_(2);
            x_(0) = ro  * cos(phi);
            x_(1) = ro  * sin(phi);
        }

        // done initializing, no need to predict or update
        is_initialized_ = true;

        return;
       }

       /*****************************************************************************
       *  Prediction
       ****************************************************************************/
       // compute the time elapsed between the current and previous measurements
       float dt = (meas_package.timestamp_ - time_us_)/ 1000000.0;  //dt-expressed in seconds
       time_us_ = meas_package.timestamp_;

       Prediction(dt);

       /*****************************************************************************
       *  Update
       ****************************************************************************/
       
       if(meas_package.sensor_type_ == MeasurementPackage::LASER){
         UpdateLidar(meas_package);
       }
       else if(meas_package.sensor_type_ == MeasurementPackage::RADAR){
         UpdateRadar(meas_package);
       }
      }
}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {
  /**
  TODO:

  Complete this function! Estimate the object's location. Modify the state
  vector, x_. Predict sigma points, the state, and the state covariance matrix.
  */

   /*****************************************************************************
  *  Generate Sigma Points
  ****************************************************************************/
  
  //create sigma point matrix
  MatrixXd Xsig = MatrixXd(n_x_, 2*n_x_ +1);

  //calculate square root of P
  MatrixXd A = P_.llt().matrixL();

  //set lambda for non-augmented sigma points
  lambda_ = 3 - n_x_;

  //set firt column of sigma point matrix
  Xsig.col(0) = x_;

  //set reamaining sigma points
  for(int i = 0; i < n_x_; i++){
    Xsig.col(i+1) = x_ + sqrt(lambda_ + n_x_) * A.col(i);
    Xsig.col(i+1 + n_x_) = x_ - sqrt(lambda_ + n_x_)* A.col(i);
  }

  /*****************************************************************************
  *  Augment Sigma Points
  ****************************************************************************/
  //create augmented mean vector
  VectorXd x_aug = VectorXd(n_aug_);

  //create augemented state covariance
  MatrixXd P_aug = MatrixXd(n_aug_, n_aug_);

  //create sigma point matrix 
  MatrixXd Xsig_aug = MatrixXd(n_aug_, 2*n_aug_ + 1);

  //set lambda for augmented sigma points
  lambda_ = 3 - n_aug_;

  //create augmented mean state
  x_aug.head(5) = x_;
  x_aug(5) = 0;
  x_aug(6) = 0;

  //create augmented covariance matrix
  P_aug.fill(0.0);
}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use lidar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the lidar NIS.
  */
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use radar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the radar NIS.
  */
}
