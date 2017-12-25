#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>
#include <fstream>

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
    std_a_ = 9.8/8;

  // Process noise standard deviation yaw acceleration in rad/s^2
    std_yawdd_ = M_PI/4;

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

  // Parameters above this line are scaffolding, do not modify
  
  /**
  TODO:

  Complete the initialization. See ukf.h for other member properties.

  Hint: one or more values initialized above might be wildly off...
  */
    n_x_ = 5;
    n_aug_ = 7;
    n_sig_ = 2*n_aug_ + 1;
    lambda_ = 3 - n_aug_;
    weights_ = VectorXd(n_sig_);
    double weight = lambda_/(lambda_+n_aug_);
    weights_(0) = weight;
    for (int i=1; i < n_sig_; i++)
    {
        weight = 0.5/(lambda_+n_aug_);
        weights_(i) = weight;
    }

    Xsig_pred_ = MatrixXd(n_x_,n_sig_);
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

    /*****************************************************************************
     *  Initialization
     ****************************************************************************/
    if (!is_initialized_) {
        /**
         TODO:
         * Initialize the state ekf_.x_ with the first measurement.
         * Create the covariance matrix.
         * Remember: you'll need to convert radar from polar to cartesian coordinates.
         */

        // first measurement
        cout << "UKF: " << endl;

        P_ = MatrixXd::Identity(5,5);
        
        if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
            /**
             Convert radar from polar to cartesian coordinates and initialize state.
             */
            float rho = meas_package.raw_measurements_[0];
            float phi = meas_package.raw_measurements_[1];
            float rho_d = meas_package.raw_measurements_[2];
            float px = rho*cos(phi);
            float py = rho*sin(phi);
            float vx = 0;
            float vy = 0;
            float v = 0;
            float psi = 0;
            float psi_d = 0;
            x_ << px,py,v, psi, psi_d;
        }
        else if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
            /**
             Initialize state.
             */
            float px = meas_package.raw_measurements_[0];
            float py = meas_package.raw_measurements_[1];
            float v = 0.0;
            float psi = 0;
            float psi_d = 0;
            x_ << px,py,v,psi,psi_d;
        }
        
        previous_timestamp_ = meas_package.timestamp_;
        std::cout << "Initialization Succeeded!!!" << std::endl;
        // done initializing, no need to predict or update
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
     * Update the process noise covariance matrix.
     * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
     */
    
    //compute the time elapsed between the current and previous measurements

    bool bool1 = use_radar_ && (meas_package.sensor_type_ == MeasurementPackage::RADAR);
    bool bool2 = use_laser_ && (meas_package.sensor_type_ == MeasurementPackage::LASER);
    
    if (bool1 || bool2)
    {
        double dt = (meas_package.timestamp_ - previous_timestamp_) / 1000000.0;    //dt - expressed in seconds
        previous_timestamp_ = meas_package.timestamp_;

        if (dt > 0)
        {
            Prediction(dt);
        }
        /*****************************************************************************
         *  Update
         ****************************************************************************/
        
        /**
         TODO:
         * Use the sensor type to perform the update step.
         * Update the state and covariance matrices.
         */
        
        if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
            // Radar updates
            UpdateRadar(meas_package);
        } else if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
            // Laser updates
            UpdateLidar(meas_package);
        }

        // print the output
        cout << "x_ = " << x_ << endl;
        cout << "P_ = " << P_ << endl;
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

    double dt2 = delta_t*delta_t;

    //predict sigma points
    MatrixXd Xsig_aug = GenerateSigmaPoints();

    //avoid division by zero
    //write predicted sigma points into right column
    VectorXd x_next = SigmaPointPrediction(delta_t,Xsig_aug);

    MatrixXd P_next = MatrixXd(n_x_,n_x_);
    P_next.fill(0.0);
    for (int i=0; i<n_sig_;i++)
    {
        VectorXd del_x = Xsig_pred_.col(i)-x_next;
        while (del_x(3)> M_PI) del_x(3)-=2.*M_PI;
        while (del_x(3)<-M_PI) del_x(3)+=2.*M_PI;
//        while (del_x(4)> M_PI) del_x(4)-=2.*M_PI;
//        while (del_x(4)<-M_PI) del_x(4)+=2.*M_PI;
        P_next += weights_(i)*del_x*del_x.transpose();
    }
    x_ = x_next;
    P_ = P_next;
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
    int n_z = 2;
    VectorXd z = VectorXd(n_z);
    z << meas_package.raw_measurements_[0], meas_package.raw_measurements_[1];
    
    MatrixXd Zsig = MatrixXd(n_z,n_sig_);
    VectorXd z_pred = VectorXd(n_z);
    z_pred.fill(0.0);
    for (int i=0; i < n_sig_; i++)
    {
        VectorXd x = Xsig_pred_.col(i);
        VectorXd z1 = VectorXd(n_z);
        
        z1(0) = x(0);
        z1(1) = x(1);

        Zsig.col(i) = z1;
        z_pred += weights_(i)*z1;
    }
    MatrixXd Tc = MatrixXd::Zero(n_x_,n_z);
    
    MatrixXd R = MatrixXd::Zero(n_z,n_z);
    R(0,0) = std_laspx_*std_laspx_;
    R(1,1) = std_laspy_*std_laspy_;
    
    MatrixXd S = R;
    for (int i=0; i<n_sig_;i++)
    {
        VectorXd delta_z = Zsig.col(i)-z_pred;
        VectorXd delta_x = Xsig_pred_.col(i)-x_;
        
        while (delta_x(3)> M_PI) delta_x(3)-=2.*M_PI;
        while (delta_x(3)<-M_PI) delta_x(3)+=2.*M_PI;
        
//        while (delta_x(4)> M_PI) delta_x(4)-=2.*M_PI;
//        while (delta_x(4)<-M_PI) delta_x(4)+=2.*M_PI;
        
        S += weights_(i)*delta_z*delta_z.transpose();
        Tc += weights_(i)*delta_x*delta_z.transpose();
    }
    MatrixXd K = Tc*S.inverse();
    VectorXd z_diff =z-z_pred;
    while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
    while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;
    x_ += K*(z_diff);
    while (x_(3)> M_PI) x_(3)-=2.*M_PI;
    while (x_(3)<-M_PI) x_(3)+=2.*M_PI;
//    while (x_(4)> M_PI) x_(4)-=2.*M_PI;
//    while (x_(4)<-M_PI) x_(4)+=2.*M_PI;
    P_ -= K*S*K.transpose();
    double nis_lidar = (z_diff).transpose()*S.inverse()*(z_diff);
    ofstream fout;
    fout.open ("nis_stdAcc_" + std::to_string(std_a_) + "_stdYawDD_" + std::to_string(std_yawdd_) + ".csv",ios::app);
    fout << nis_lidar << std::endl;
    fout.close();
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
    int n_z = 3;
    VectorXd z = VectorXd(n_z);
    z << meas_package.raw_measurements_[0], meas_package.raw_measurements_[1], meas_package.raw_measurements_[2];

    MatrixXd Zsig = MatrixXd(n_z,n_sig_);
    VectorXd z_pred = VectorXd(n_z);
    z_pred.fill(0.0);
    for (int i=0; i < n_sig_; i++)
    {
        VectorXd x = Xsig_pred_.col(i);
        VectorXd z1 = VectorXd(n_z);
        z1(0) = sqrt(x(0)*x(0) + x(1)*x(1));
        z1(1) = atan2(x(1),x(0));
        if (fabs(x(0)) < 1E-6 && fabs(x(1)) < 1E-6)
        {
            z1(2) = 0;
        }
        else
        {
            z1(2) = (x(0)*cos(x(3))*x(2) + x(1)*sin(x(3))*x(2))/z1(0);
//            z1(2) = (x(0)*sin(x(3))*x(2) - x(1)*cos(x(3))*x(2))/(z1(0)*z1(0));
        }
        Zsig.col(i) = z1;
        z_pred += weights_(i)*z1;
    }
    MatrixXd Tc = MatrixXd::Zero(n_x_,n_z);

    MatrixXd R = MatrixXd::Zero(n_z,n_z);
    R(0,0) = std_radr_*std_radr_;
    R(1,1) = std_radphi_*std_radphi_;
    R(2,2) = std_radrd_*std_radrd_;

    MatrixXd S = R;
    for (int i=0; i<n_sig_;i++)
    {
        VectorXd delta_z = Zsig.col(i)-z_pred;
        VectorXd delta_x = Xsig_pred_.col(i)-x_;
        while (delta_z(1)> M_PI) delta_z(1)-=2.*M_PI;
        while (delta_z(1)<-M_PI) delta_z(1)+=2.*M_PI;
        
        while (delta_x(3)> M_PI) delta_x(3)-=2.*M_PI;
        while (delta_x(3)<-M_PI) delta_x(3)+=2.*M_PI;
//        while (delta_x(4)> M_PI) delta_x(4)-=2.*M_PI;
//        while (delta_x(4)<-M_PI) delta_x(4)+=2.*M_PI;
        
        S += weights_(i)*delta_z*delta_z.transpose();
        Tc += weights_(i)*delta_x*delta_z.transpose();
    }
    MatrixXd K = Tc*S.inverse();
    VectorXd z_diff =z-z_pred;
    while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
    while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;
    x_ += K*(z_diff);
    while (x_(3)> M_PI) x_(3)-=2.*M_PI;
    while (x_(3)<-M_PI) x_(3)+=2.*M_PI;
//    while (x_(4)> M_PI) x_(4)-=2.*M_PI;
//    while (x_(4)<-M_PI) x_(4)+=2.*M_PI;
    P_ -= K*S*K.transpose();
    double nis_radar = (z_diff).transpose()*S.inverse()*(z_diff);
    ofstream fout;
    fout.open ("nis_stdAcc_" + std::to_string(std_a_) + "_stdYawDD_" + std::to_string(std_yawdd_) + ".csv",ios::app);
    fout << nis_radar << std::endl;
    fout.close();
}

MatrixXd UKF::GenerateSigmaPoints()
{
    VectorXd x_st = VectorXd(n_aug_);
    x_st.head(n_x_) = x_;
    x_st(n_x_) = 0;//std_a_*std_a_;
    x_st(n_x_+1) = 0;//std_yawdd_*std_yawdd_;
    
    MatrixXd Q_noise = MatrixXd(2,2);
    Q_noise << std_a_*std_a_,         0,
                    0     ,std_yawdd_*std_yawdd_;
    MatrixXd P_aug = MatrixXd(n_aug_,n_aug_);
    P_aug.fill(0.0);
    P_aug.topLeftCorner(n_x_, n_x_) = P_;
    P_aug(n_x_, n_x_) = std_a_*std_a_;
    P_aug(n_x_+1, n_x_+1) = std_yawdd_*std_yawdd_;

    //create sigma point matrix
    MatrixXd Xsig = MatrixXd(n_aug_, n_sig_);
    
    //calculate square root of P
    MatrixXd A = P_aug.llt().matrixL();

    //calculate sigma points ...
    //set sigma points as columns of matrix Xsig
    Xsig.col(0) = x_st;
    
    for (int i=0;i<A.cols();i++)
    {
        VectorXd x1 = x_st + sqrt(lambda_+n_aug_)*A.col(i);
        VectorXd x2 = x_st - sqrt(lambda_+n_aug_)*A.col(i);
        
//        while (x1(3)> M_PI) x1(3)-=2.*M_PI;
//        while (x1(3)<-M_PI) x1(3)+=2.*M_PI;
//        while (x1(4)> M_PI) x1(4)-=2.*M_PI;
//        while (x1(4)<-M_PI) x1(4)+=2.*M_PI;
//
//        while (x2(3)> M_PI) x2(3)-=2.*M_PI;
//        while (x2(3)<-M_PI) x2(3)+=2.*M_PI;
//        while (x2(4)> M_PI) x2(4)-=2.*M_PI;
//        while (x2(4)<-M_PI) x2(4)+=2.*M_PI;
        
        Xsig.col(i+1) = x1;
        Xsig.col(i+1+n_aug_) = x2;
    }
    
    //write result
    return Xsig;
}

VectorXd UKF::SigmaPointPrediction(double delta_t, MatrixXd Xsig_aug)
{
    MatrixXd Xsig_pred = MatrixXd(n_x_,n_sig_);
    
    double dt2 = delta_t*delta_t;
    
    VectorXd x_next = VectorXd(x_.size());
    x_next.fill(0.0);
    
    for(int i=0; i < Xsig_aug.cols();i++)
    {
        VectorXd x_state = (Xsig_aug.col(i)).head(5);//.block(0,i,5,0);
        VectorXd noise = (Xsig_aug.col(i)).tail(2);//.block(4,i,2,0);

        VectorXd vec_1 = VectorXd(x_state.size());
        VectorXd vec_2 = VectorXd(x_state.size());
        
        vec_2(0) = noise(0)*cos(x_state(3))*dt2/2;
        vec_2(1) = noise(0)*sin(x_state(3))*dt2/2;
        vec_2(2) = noise(0)*delta_t;
        vec_2(3) = noise(1)*dt2/2;
        vec_2(4) = noise(1)*delta_t;
        
        if (fabs(x_state(4)) < 1E-6)
        {
            vec_1(0) = x_state(2)*cos(x_state(3))*delta_t;
            vec_1(1) = x_state(2)*sin(x_state(3))*delta_t;
        }
        else
        {
            double phi_k1 = x_state(3) + x_state(4)*delta_t;
            vec_1(0) = x_state(2)/x_state(4)*( sin(phi_k1) - sin(x_state(3)));
            vec_1(1) = x_state(2)/x_state(4)*(-cos(phi_k1) + cos(x_state(3)));
        }
        vec_1(2) = 0;
        vec_1(3) = x_state(4)*delta_t;
        vec_1(4) = 0;
        VectorXd x1 = x_state + vec_1 + vec_2;
//        while (x1(3)> M_PI) x1(3)-=2.*M_PI;
//        while (x1(3)<-M_PI) x1(3)+=2.*M_PI;
//        while (x1(4)> M_PI) x1(4)-=2.*M_PI;
//        while (x1(4)<-M_PI) x1(4)+=2.*M_PI;
        Xsig_pred.col(i) = x1;
        x_next += weights_(i)*Xsig_pred.col(i);
        
    }
//    while (x_next(3)> M_PI) x_next(3)-=2.*M_PI;
//    while (x_next(3)<-M_PI) x_next(3)+=2.*M_PI;
//    while (x_next(4)> M_PI) x_next(4)-=2.*M_PI;
//    while (x_next(4)<-M_PI) x_next(4)+=2.*M_PI;

    //write result
    Xsig_pred_ = Xsig_pred;
    return x_next;
}
