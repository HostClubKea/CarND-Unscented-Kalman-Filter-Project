#include "LidarKalmanFilter.h"
#include <iostream>

using namespace std;
using Eigen::VectorXd;
using Eigen::MatrixXd;

LidarKalmanFilter::~LidarKalmanFilter() {}

LidarKalmanFilter::LidarKalmanFilter() {

    // Laser measurement noise standard deviation position1 in m
    std_laspx_ = 0.15;

    // Laser measurement noise standard deviation position2 in m
    std_laspy_ = 0.15;

    NIS_top = 5.99;

    NIS_bot = 0.1;

    // initializing matrices
    H_ = MatrixXd(2, n_x_);
    H_ << 1, 0, 0, 0, 0,
          0, 1, 0, 0, 0;

    //measurement covariance matrix - laser
    R_ = MatrixXd(2, 2);
    R_ << pow(std_laspx_, 2), 0,
          0, pow(std_laspy_, 2);
}

bool LidarKalmanFilter::IsProcessible(const MeasurementPackage &measurement_pack) {
    return measurement_pack.sensor_type_ == MeasurementPackage::LASER;
}

void LidarKalmanFilter::Init(FilterState &state, const MeasurementPackage &measurement_pack) {
    //Init filter state
    state.x_ << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], 0, 0, 0;
    AbstractKalmanFilter::Init(state, measurement_pack);
}

void LidarKalmanFilter::Update(FilterState &state, const VectorXd &z) {
    VectorXd z_pred = H_ * state.x_;

    VectorXd y = z - z_pred;

    MatrixXd Ht = H_.transpose();
    MatrixXd S = H_ * state.P_ * Ht + R_;
    MatrixXd Si = S.inverse();
    MatrixXd PHt = state.P_ * Ht;
    MatrixXd K = PHt * Si;

    //new estimate
    state.x_ = state.x_ + (K * y);
    long x_size = state.x_.size();
    MatrixXd I = MatrixXd::Identity(x_size, x_size);
    state.P_ = (I - K * H_) * state.P_;

    NIS_ = y.transpose() * Si * y;

}
