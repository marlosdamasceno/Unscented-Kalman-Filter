#include "ukf.h"
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
    // if this is false, laser measurements will be ignored (except for init)
    use_laser_ = true;

    // if this is false, radar measurements will be ignored (except for init)
    use_radar_ = true;

    // initial state vector
    x_ = VectorXd(5);
    x_.fill(0.0);

    // initial covariance matrix
    P_ = MatrixXd(5, 5);
    P_.fill(0.0);

    // Process noise standard deviation longitudinal acceleration in m/s^2
    std_a_ = 1.0;

    // Process noise standard deviation yaw acceleration in rad/s^2
    std_yawdd_ = 0.5;

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

    /**
     * Complete the initialization. See ukf.h for other member properties.
     * Hint: one or more values initialized above might be wildly off...
     */
    // initially set to false, set to true in first call of ProcessMeasurement
    is_initialized_ = false;

    // previous time, in us
    previous_timestamp_ = 0;

    // State dimension
    n_x_ = 5;

    // Augmented state dimension
    n_aug_ = 7;

    // Sigma point spreading parameter
    lambda_ = 3 - n_x_;

    // predicted sigma points matrix
    Xsig_pred_ = MatrixXd(n_x_, 2 * n_aug_ + 1);
    Xsig_pred_.fill(0.0);

    // Weights of sigma points
    weights_ = VectorXd(2 * n_aug_ + 1);
    weights_.fill(0.0);

    //NIS of radar and lidar
    Radar_NIS_ = 0.0;
    Lidar_NIS_ = 0.0;
}

UKF::~UKF() = default;

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage measurement_pack) {
    /*****************************************************************************
       *  Initialization
       ****************************************************************************/
    if (!is_initialized_) {
        /** * Initialize the state x_ with the first measurement.
            * Create the covariance matrix.
            * Remember: you'll need to convert radar from polar to cartesian coordinates.*/
        // first measurement
        cout << "Start UKF" << endl;
        if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR && use_radar_) {
            /** Convert radar from polar to cartesian coordinates and initialize state.*/
            x_(0) = measurement_pack.raw_measurements_[0] * cos(measurement_pack.raw_measurements_[1]); //rho * cos(phi)
            x_(1) = measurement_pack.raw_measurements_[0] * sin(measurement_pack.raw_measurements_[1]); //rho * sin(phi)
            x_(2) = 0;
            x_(3) = 0;//measurement_pack.raw_measurements_[2] * cos(measurement_pack.raw_measurements_[1]); //rho's velocity * cos(phi)
            x_(4) = 0;//measurement_pack.raw_measurements_[2] * sin(measurement_pack.raw_measurements_[1]); //rho's velocity * sin(phi)
        }
        else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER && use_laser_) {
            /** Initialize state.*/
            //Lesson 6. Section 13.
            x_(0) = measurement_pack.raw_measurements_[0]; //x
            x_(1) = measurement_pack.raw_measurements_[1]; //y
            x_(2) = 0;
            x_(3) = 0;
            x_(4) = 0;
        }
        //Changing the P, from 1 to 0.01 and from 1000 to 1, generates a much lower RMSE
        P_ << std_radr_*std_radr_, 0, 0, 0, 0,
                0, std_radr_*std_radr_, 0, 0, 0,
                0, 0, 1, 0, 0,
                0, 0, 0, std_radphi_, 0,
                0, 0, 0, 0, std_radphi_;


        previous_timestamp_ =  measurement_pack.timestamp_;
        // done initializing, no need to predict or update
        is_initialized_ = true;
        return;
    }
    //Lesson 6. Section 13.
    double dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
    Prediction(dt);

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR && use_radar_) {
        UpdateRadar(measurement_pack);
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER && use_laser_) {
        UpdateLidar(measurement_pack);
    }
    previous_timestamp_ =  measurement_pack.timestamp_;
    // print the output
    cout << "x_ = " << x_ << endl;
    cout << "P_ = " << P_ << endl;
}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {
    /// Lesson 8 Section 18
    VectorXd x_aug = VectorXd(n_aug_);
    x_aug.fill(0.0);
    MatrixXd P_aug = MatrixXd(n_aug_, n_aug_);
    P_aug.fill(0.0);
    //create sigma point matrix
    MatrixXd Xsig_aug = MatrixXd(n_aug_, 2 * n_aug_ + 1);
    Xsig_aug.fill(0.0);

    //create augmented mean state
    x_aug.head(n_x_) = x_; //This line gives an error in my IDE
    x_aug(n_x_) = 0;
    x_aug(n_x_+1) = 0;

    //create augmented covariance matrix
    P_aug.topLeftCorner(n_x_,n_x_) = P_;
    P_aug(n_x_,n_x_) = std_a_*std_a_;
    P_aug(n_x_+1,n_x_+1) = std_yawdd_*std_yawdd_;

    //create square root matrix
    MatrixXd L = P_aug.llt().matrixL();
    //create augmented sigma points
    Xsig_aug.col(0)  = x_aug;
    for (int i = 0; i< n_aug_; i++)
    {
        Xsig_aug.col(i+1) = x_aug + sqrt(lambda_+n_aug_) * L.col(i);
        Xsig_aug.col(i+1+n_aug_) = x_aug - sqrt(lambda_+n_aug_) * L.col(i);
    }

    /// Lesson 8 Section 21
    //predict sigma points
    for (int i = 0; i< 2*n_aug_+1; i++)
    {
        //extract values for better readability
        double p_x = Xsig_aug(0,i);
        double p_y = Xsig_aug(1,i);
        double v = Xsig_aug(2,i);
        double yaw = Xsig_aug(3,i);
        double yawd = Xsig_aug(4,i);
        double nu_a = Xsig_aug(5,i);
        double nu_yawdd = Xsig_aug(6,i);

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

    /// Lesson 8 Section 24
    // set weights
    double weight_0 = lambda_/(lambda_+n_aug_);
    weights_(0) = weight_0;
    for (int i=1; i<2*n_aug_+1; i++) {  //2n+1 weights
        double weight = 0.5/(n_aug_+lambda_);
        weights_(i) = weight;
    }

    //predicted state mean
    x_.fill(0.0);
    for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //iterate over sigma points
        x_ = x_ + weights_(i) * Xsig_pred_.col(i);
    }

    //predicted state covariance matrix
    P_.fill(0.0);
    for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //iterate over sigma points

        // state difference
        VectorXd x_diff = Xsig_pred_.col(i) - x_;
        //angle normalization
        while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
        while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;

        P_ = P_ + weights_(i) * x_diff * x_diff.transpose() ;
    }
}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {
    /**
     * Complete this function! Use lidar data to update the belief about the object's
     * position. Modify the state vector, x_, and covariance, P_.
     * You'll also need to calculate the lidar NIS.
    */
    //H matrix for laser. Lesson 6. Section 10.
    MatrixXd H_ = MatrixXd(2, 5);
    //measurement covariance matrix - laser
    MatrixXd R_ = MatrixXd(2, 2);
    H_ <<   1, 0, 0, 0, 0,
            0, 1, 0, 0, 0;
    R_ <<   (std_laspx_*std_laspx_), 0,
            0, (std_laspy_*std_laspy_);

    //Similar to Lesson 6. Section 7.
    VectorXd z_pred = H_ * x_;
    VectorXd z = VectorXd(2);
    z(0) = meas_package.raw_measurements_(0); // x
    z(1) = meas_package.raw_measurements_(1); // y

    VectorXd z_diff = z - z_pred; // innovation (residual error)
    MatrixXd S = H_* P_ * H_.transpose() + R_; //This line gives me an error on my IDE
    MatrixXd K = P_ * H_.transpose() * S.inverse();
    x_ = x_ + (K * z_diff);

    long x_size = x_.size();
    MatrixXd I = MatrixXd::Identity(x_size, x_size);
    P_ = (I - K * H_) * P_;

    Lidar_NIS_ = z_diff.transpose() * S.inverse() * z_diff;
    std::cout << "Lidar NIS: " << Lidar_NIS_ << std::endl;
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {
    /**
     * Complete this function! Use radar data to update the belief about the object's
     * position. Modify the state vector, x_, and covariance, P_.
     * You'll also need to calculate the radar NIS.
    */
    /// Lesson 8 Section 27
    int n_z_ = 3;
    MatrixXd Zsig = MatrixXd(n_z_, 2 * n_aug_ + 1);
    Zsig.fill(0.0);
    //transform sigma points into measurement space
    for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points
        // extract values for better readibility
        double p_x = Xsig_pred_(0,i);
        double p_y = Xsig_pred_(1,i);
        double v  = Xsig_pred_(2,i);
        double yaw = Xsig_pred_(3,i);
        double v1 = cos(yaw)*v;
        double v2 = sin(yaw)*v;

        // measurement model
        Zsig(0,i) = sqrt(p_x*p_x + p_y*p_y);                        //r
        Zsig(1,i) = atan2(p_y,p_x);                                 //phi
        Zsig(2,i) = (p_x*v1 + p_y*v2 ) / sqrt(p_x*p_x + p_y*p_y);   //r_dot
    }

    //mean predicted measurement
    VectorXd z_pred = VectorXd(n_z_);
    z_pred.fill(0.0);
    for (int i=0; i < 2*n_aug_+1; i++) {
        z_pred = z_pred + weights_(i) * Zsig.col(i);
    }

    //innovation covariance matrix S
    MatrixXd S = MatrixXd(n_z_,n_z_);
    S.fill(0.0);
    for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points
        //residual
        VectorXd z_diff = Zsig.col(i) - z_pred;

        //angle normalization
        while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
        while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

        S = S + weights_(i) * z_diff * z_diff.transpose();
    }

    //add measurement noise covariance matrix
    MatrixXd R = MatrixXd(n_z_,n_z_);
    R <<    std_radr_*std_radr_, 0, 0,
            0, std_radphi_*std_radphi_, 0,
            0, 0,std_radrd_*std_radrd_;
    S = S + R;

    /// Lesson 8 Section 30
     MatrixXd Tc = MatrixXd(n_x_, n_z_);
    //calculate cross correlation matrix
    Tc.fill(0.0);
    for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points

        //residual
        VectorXd z_diff = Zsig.col(i) - z_pred;
        //angle normalization
        while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
        while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

        // state difference
        VectorXd x_diff = Xsig_pred_.col(i) - x_;
        //angle normalization
        while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
        while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;

        Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
    }

    //Kalman gain K;
    MatrixXd K = Tc * S.inverse();

    //residual
    //create example vector for incoming radar measurement
    VectorXd z = VectorXd(n_z_);
    z <<    meas_package.raw_measurements_(0),
            meas_package.raw_measurements_(1),
            meas_package.raw_measurements_(2);
    VectorXd z_diff = z - z_pred;

    //angle normalization
    while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
    while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

    //update state mean and covariance matrix
    x_ = x_ + K * z_diff;
    P_ = P_ - K*S*K.transpose();

    //calculate NIS
    Radar_NIS_ = z_diff.transpose() * S.inverse() * z_diff;
    std::cout << "Radar NIS: " << Lidar_NIS_ << std::endl;
}
