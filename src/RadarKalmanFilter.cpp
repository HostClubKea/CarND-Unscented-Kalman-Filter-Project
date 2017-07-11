#include "RadarKalmanFilter.h"
#include <iostream>
#include "tools.h"

using namespace std;
using Eigen::MatrixXd;

RadarKalmanFilter::~RadarKalmanFilter() {}

RadarKalmanFilter::RadarKalmanFilter() {
    n_z_ = 3;

    // Radar measurement noise standard deviation radius in m
    std_radr_ = 0.3;

    // Radar measurement noise standard deviation angle in rad
    std_radphi_ = 0.03;

    // Radar measurement noise standard deviation radius change in m/s
    std_radrd_ = 0.3;

    // initializing matrices
    R_ = MatrixXd(3, 3);
    R_ << pow(std_radr_, 2), 0, 0,
          0, pow(std_radphi_, 2), 0,
          0, 0, pow(std_radrd_, 2);

    NIS_top = 7.8;

    NIS_bot = 0.35;

    eps = 0.000001;
}

void RadarKalmanFilter::Update(FilterState &state, const VectorXd &z) {
    MatrixXd Zsig = TransformSigmaPointsToMeasurementSpace();

    //mean predicted measurement
    VectorXd z_pred = ComputeMean(Zsig, n_z_);

    //measurement covariance matrix S
    MatrixXd S = ComputeCovariance(Zsig, z_pred, 1, n_z_);

    //add measurement noise covariance matrix
    S = S + R_;

    //calculate cross correlation matrix
    MatrixXd Tc = CalculateCrossCorrelation(state, Zsig, z_pred);

    //Kalman gain K;
    MatrixXd K = Tc * S.inverse();

    //residual
    VectorXd z_diff = z - z_pred;

    //angle normalization
    while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
    while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

    //update state mean and covariance matrix
    state.x_ = state.x_ + K * z_diff;
    state.P_ = state.P_ - K*S*K.transpose();

    NIS_ = z_diff.transpose() * S.transpose() * z_diff;
}

MatrixXd RadarKalmanFilter::CalculateCrossCorrelation(const FilterState &state, MatrixXd &Zsig, const VectorXd &z_pred) const {//create matrix for cross correlation Tc
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
        VectorXd x_diff = Xsig_pred_.col(i) - state.x_;
        //angle normalization
        while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
        while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;

        Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
    }
    return Tc;
}

MatrixXd RadarKalmanFilter::TransformSigmaPointsToMeasurementSpace() const {//create matrix for sigma points in measurement space
    MatrixXd Zsig = MatrixXd(n_z_, 2 * n_aug_ + 1);

    //transform sigma points into measurement space
    for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points

        // extract values for better readibility
        double p_x = Xsig_pred_(0, i);
        double p_y = Xsig_pred_(1, i);
        double v  = Xsig_pred_(2, i);
        double yaw = Xsig_pred_(3, i);

        double v1 = cos(yaw)*v;
        double v2 = sin(yaw)*v;

        // measurement model
        Zsig(0,i) = sqrt(p_x*p_x + p_y*p_y);                        //r
        Zsig(1,i) = atan2(p_y,p_x);                                 //phi
        Zsig(2,i) = (p_x*v1 + p_y*v2 ) / sqrt(p_x*p_x + p_y*p_y);   //r_dot
    }
    return Zsig;
}


bool RadarKalmanFilter::IsProcessible(const MeasurementPackage &measurement_pack) {
    return measurement_pack.sensor_type_ == MeasurementPackage::RADAR;
}

void RadarKalmanFilter::Init(FilterState &state, const MeasurementPackage &measurement_pack) {

    double ro = measurement_pack.raw_measurements_[0]; // range
    double theta = measurement_pack.raw_measurements_[1]; // bearing
    double ro_dot = measurement_pack.raw_measurements_[2]; // velocity of rho
    //Convert radar from polar to cartesian coordinates
    double x = ro * cos(theta);
    double y = ro * sin(theta);
    state.x_ << x, y, ro_dot, theta, 0;

    AbstractKalmanFilter::Init(state, measurement_pack);
}
