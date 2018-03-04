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
  std_a_ = 6;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 1;
  
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
  n_x_ = 5;

  n_aug_ = 7;

  n_radar_ = 3;

  n_laser_ = 2;

  lambda_ = 3 - n_aug_;

  weights_ = VectorXd( 2*n_aug_+1 );
  weights_(0) = lambda_/( lambda_ + n_aug_ );
  for( int i=1; i<2*n_aug_+1; i++ )
    weights_(i) = 0.5/( n_aug_ + lambda_ );

  R_laser_ = MatrixXd(n_laser_, n_laser_);
  R_laser_ << std_laspx_*std_laspx_ , 0,
      0, std_laspy_*std_laspy_;

  R_radar_ = MatrixXd(n_radar_, n_radar_);
  R_radar_ << std_radr_*std_radr_ , 0, 0,
      0, std_radphi_*std_radphi_, 0,
      0, 0, std_radrd_*std_radrd_;

  Xsig_pred_ = MatrixXd(n_x_, 2*n_aug_+1);

  nis_laser_.open( "./NISvals_laser.txt", ios::out );
  nis_radar_.open( "./NISvals_radar.txt", ios::out );

  if (!nis_laser_.is_open()){
    cout << "Failed to open NISvals_laser.txt." <<endl;
  }

  if (!nis_radar_.is_open()){
    cout << "Failed to open NISvals_radar.txt." <<endl;
  }
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
  if (!is_initialized_)
  {
    // Initialize the state ekf_.x_ with the first measurement.
    // Create the covariance matrix.

    cout << "Initializing unscented Kalman filter" << endl;

    if( meas_package.sensor_type_ == MeasurementPackage::RADAR )
    {
      // Convert radar from polar to cartesian coordinates and initialize state.
      double rho = meas_package.raw_measurements_[0];
      double phi = meas_package.raw_measurements_[1];
      x_ << rho*cos(phi), rho*sin(phi), 0., 0., 0.;
    }
    else if( meas_package.sensor_type_ == MeasurementPackage::LASER )
    {
      // Initialize state.
      x_ << meas_package.raw_measurements_[0], meas_package.raw_measurements_[1], 0., 0., 0.;
    }

    previous_timestamp_ = meas_package.timestamp_;

    // Estimate the initial state covariance matrix
    // with a moderate covariance of 1 for all values.  This is somewhat
    // unrealistic because the velocity, yaw angle, and yaw rate are completely
    // unknown from the initial measurements.
    P_.fill(0.);
    P_(0,0) = 1.;
    P_(1,1) = 1.;
    P_(2,2) = 1.;
    P_(3,3) = 1.;
    P_(4,4) = 1.;

    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  // Compute the time from the previous measurement in seconds.
  double dt = ( meas_package.timestamp_ - previous_timestamp_ )/1000000.0;
  previous_timestamp_ = meas_package.timestamp_;

  Prediction( dt );

  if( meas_package.sensor_type_ == MeasurementPackage::RADAR && use_radar_)
  {
    // Radar update
    UpdateRadar( meas_package );
  }
  else if(meas_package.sensor_type_ == MeasurementPackage::LASER && use_laser_)
  {
    // Laser update
    UpdateLidar( meas_package );
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
  MatrixXd Xsig_aug_, P_aug_;
  VectorXd x_aug;

  Xsig_aug_= MatrixXd( n_aug_, 2*n_aug_+1 );
  P_aug_ = MatrixXd( n_aug_, n_aug_ );
  Xsig_pred_ = MatrixXd( n_x_, 2*n_aug_+1 );
  x_aug = VectorXd(n_aug_);

  x_aug.head(5) = x_;
  x_aug(5) = 0;
  x_aug(6) = 0;

  //create augmented covariance matrix
  P_aug_.fill(0.0);
  P_aug_.topLeftCorner(5,5) = P_;
  P_aug_(5,5) = std_a_*std_a_;
  P_aug_(6,6) = std_yawdd_*std_yawdd_;

  //create square root matrix
  MatrixXd L = P_aug_.llt().matrixL();

  //create augmented sigma points
  Xsig_aug_.col(0)  = x_aug;
  for (int i = 0; i< n_aug_; i++)
  {
    Xsig_aug_.col(i+1)       = x_aug + sqrt(lambda_+n_aug_) * L.col(i);
    Xsig_aug_.col(i+1+n_aug_) = x_aug - sqrt(lambda_+n_aug_) * L.col(i);
  }

  //predict sigma points
  for (int i = 0; i< 2*n_aug_+1; i++)
  {
    //extract values for better readability
    double p_x = Xsig_aug_(0,i);
    double p_y = Xsig_aug_(1,i);
    double v = Xsig_aug_(2,i);
    double yaw = Xsig_aug_(3,i);
    double yawd = Xsig_aug_(4,i);
    double nu_a = Xsig_aug_(5,i);
    double nu_yawdd = Xsig_aug_(6,i);

    //predicted state values
    double px_p, py_p;

    //avoid division by zero
    if (fabs(yawd) > 0.001) {
      px_p = p_x + v/yawd * ( sin (yaw + yawd*delta_t) - sin(yaw));
      py_p = p_y + v/yawd * ( cos(yaw) - cos(yaw+yawd*delta_t) );
    }
    else {
      px_p = p_x + v*delta_t*cos(yaw);
      py_p = p_y + v*delta_t*sin(yaw);
    }

    double v_p = v;
    double yaw_p = yaw + yawd*delta_t;
    double yawd_p = yawd;

    //add noise
    px_p = px_p + 0.5*nu_a*delta_t*delta_t * cos(yaw);
    py_p = py_p + 0.5*nu_a*delta_t*delta_t * sin(yaw);
    v_p = v_p + nu_a*delta_t;

    yaw_p = yaw_p + 0.5*nu_yawdd*delta_t*delta_t;
    yawd_p = yawd_p + nu_yawdd*delta_t;

    //write predicted sigma point into right column
    Xsig_pred_(0,i) = px_p;
    Xsig_pred_(1,i) = py_p;
    Xsig_pred_(2,i) = v_p;
    Xsig_pred_(3,i) = yaw_p;
    Xsig_pred_(4,i) = yawd_p;
  }

  //predict state mean
  x_.fill(0.);
  for( int i = 0; i < 2*n_aug_+1; i++ )
    x_ = x_ + weights_(i)*Xsig_pred_.col(i);

  //predict state covariance matrix
  P_.fill(0.);
  for( int i = 0; i < 2*n_aug_+1; i++)
  {
    VectorXd deltax_ = Xsig_pred_.col(i) - x_;
    deltax_ = NormalizeStateVector(deltax_);
    P_ = P_ + weights_(i)*deltax_*deltax_.transpose();
  }

  /*
  cout << "DEBUG: " << endl;
  cout << "X: " << x_ << endl;
  cout << "P: " << P_ << endl;
   */
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

  MatrixXd Zsig_laser_, S_laser_, T_laser_, K_laser_;
  VectorXd z_pred_laser_;

  Zsig_laser_ = MatrixXd(n_laser_, 2*n_aug_ + 1);
  z_pred_laser_ = VectorXd(n_laser_);
  S_laser_ = MatrixXd(n_laser_, n_laser_);
  T_laser_ = MatrixXd(n_x_, n_laser_);
  K_laser_ = MatrixXd(n_x_, n_laser_);

  Zsig_laser_.fill(0.);
  for( int i = 0; i < 2*n_aug_ + 1; i++ )
  {
    Zsig_laser_(0,i) = Xsig_pred_(0,i);
    Zsig_laser_(1,i) = Xsig_pred_(1,i);
  }

  //calculate mean predicted measurement
  z_pred_laser_.fill(0.);
  for( int i = 0; i < 2*n_aug_ + 1; i++ )
    z_pred_laser_ = z_pred_laser_ + weights_(i)*Zsig_laser_.col(i);

  //calculate measurement covariance matrix S_laser_
  S_laser_.fill(0.);
  for( int i = 0; i < 2*n_aug_ + 1; i++ )
  {
    VectorXd deltaz_laser_ = Zsig_laser_.col(i) - z_pred_laser_;
    S_laser_ = S_laser_ + weights_(i)*deltaz_laser_*deltaz_laser_.transpose();
  }

  S_laser_ = S_laser_ + R_laser_;

  //calculate cross correlation matrix
  T_laser_.fill(0.);
  for( int i = 0; i < 2*n_aug_ + 1; i++ )
  {
    VectorXd deltax_ = Xsig_pred_.col(i) - x_;
    VectorXd deltaz_laser_ = Zsig_laser_.col(i) - z_pred_laser_;
    T_laser_ = T_laser_ + weights_(i)*deltax_*deltaz_laser_.transpose();
  }

  //calculate K_laser_alman gain K_laser_;
  K_laser_ = T_laser_*S_laser_.inverse();

  //update state mean and covariance matrix
  VectorXd deltaz_laser_ = meas_package.raw_measurements_ - z_pred_laser_;

  x_ = x_ + K_laser_*deltaz_laser_;
  P_ = P_ - K_laser_*S_laser_*K_laser_.transpose();

  /*
  std::cout << "Debug update laser" << std::endl;
  std::cout << "delta_z " << deltaz_laser_ << std::endl;
  std::cout << "Zsig " << Zsig_laser_ << std::endl;
  std::cout << "w " << weights_ << std::endl;
  std::cout << "S " << S_laser_ << std::endl;
  std::cout << "T " << T_laser_ << std::endl;
  std::cout << "K " << K_laser_ << std::endl;
   */

  double nis = deltaz_laser_.transpose()*S_laser_.inverse()*deltaz_laser_;

  nis_laser_ << nis << endl;
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
  MatrixXd Zsig_radar_, S_radar_, T_radar_, K_radar_;
  VectorXd z_pred_radar_;

  Zsig_radar_ = MatrixXd(n_radar_, 2*n_aug_ + 1);
  z_pred_radar_ = VectorXd(n_radar_);
  S_radar_ = MatrixXd(n_radar_, n_radar_);
  T_radar_ = MatrixXd(n_x_, n_radar_);
  K_radar_ = MatrixXd(n_x_, n_radar_);

  Zsig_radar_.fill(0.);
  for( int i = 0; i < 2*n_aug_ + 1; i++ )
  {
    double px = Xsig_pred_(0,i);
    double py = Xsig_pred_(1,i);
    double v = Xsig_pred_(2,i);
    double psi = Xsig_pred_(3,i);
    Zsig_radar_(0,i) = sqrt(px*px+py*py);
    if(fabs(Zsig_radar_(0,i)) > 1e-6){
      Zsig_radar_(1,i) = atan2(py,px);
      Zsig_radar_(2,i) = ( px*cos(psi)*v + py*sin(psi)*v )/sqrt(px*px+py*py);
    }
    else{
      Zsig_radar_(1,i) = 0;
      Zsig_radar_(2,i) = 0;
    }

  }

  //calculate mean predicted measurement
  z_pred_radar_.fill(0.);
  for( int i = 0; i < 2*n_aug_ + 1; i++ )
  {
    z_pred_radar_ = z_pred_radar_ + weights_(i)*Zsig_radar_.col(i);
  }

  z_pred_radar_ = NormalizeRadarMeasurementVector(z_pred_radar_);

  //calculate measurement covariance matrix S_radar_
  S_radar_.fill(0.);
  for( int i = 0; i < 2*n_aug_ + 1; i++ )
  {
    VectorXd deltaz_radar_ = Zsig_radar_.col(i) - z_pred_radar_;
    deltaz_radar_ = NormalizeRadarMeasurementVector(deltaz_radar_);
    S_radar_ = S_radar_ + weights_(i)*deltaz_radar_*deltaz_radar_.transpose();
  }

  S_radar_ = S_radar_ + R_radar_;

  //calculate cross correlation matrix
  T_radar_.fill(0.);
  for( int i = 0; i < 2*n_aug_ + 1; i++ )
  {
    VectorXd deltax_ = Xsig_pred_.col(i) - x_;
    VectorXd deltaz_radar_ = Zsig_radar_.col(i) - z_pred_radar_;
    deltax_ = NormalizeRadarMeasurementVector(deltax_);
    deltaz_radar_ = NormalizeRadarMeasurementVector(deltaz_radar_);
    T_radar_ = T_radar_ + weights_(i)*deltax_*deltaz_radar_.transpose();
  }

  //calculate K_radar_alman gain K_radar_;
  K_radar_ = T_radar_*S_radar_.inverse();

  //update state mean and covariance matrix
  VectorXd deltaz_radar_ = meas_package.raw_measurements_ - z_pred_radar_;
  deltaz_radar_ = NormalizeRadarMeasurementVector(deltaz_radar_);

  x_ = x_ + K_radar_*deltaz_radar_;
  P_ = P_ - K_radar_*S_radar_*K_radar_.transpose();

  /*
  std::cout << "Debug" << std::endl;
  std::cout << "Zsig " << Zsig_radar_ << std::endl;
  std::cout << "w " << weights_ << std::endl;
  std::cout << "S " << S_radar_ << std::endl;
  std::cout << "T " << T_radar_ << std::endl;
  std::cout << "K " << K_radar_ << std::endl;
   */


  // Uncomment the following to print normalized innovation squared (NIS_radar_),
  // so that it can be plotted and serve as a consistency check on
  // our choice of process noise values
  double nis = deltaz_radar_.transpose()*S_radar_.inverse()*deltaz_radar_;

  nis_radar_ << nis << endl;
}

VectorXd UKF::NormalizeRadarMeasurementVector(VectorXd vector)
{
  while( vector(1)> M_PI ) vector(1)-=2.*M_PI;
  while( vector(1)<-M_PI ) vector(1)+=2.*M_PI;

  return vector;
}

VectorXd UKF::NormalizeStateVector(VectorXd vector)
{
  while( vector(3)> M_PI ) vector(3)-=2.*M_PI;
  while( vector(3)<-M_PI ) vector(3)+=2.*M_PI;

  return vector;
}
