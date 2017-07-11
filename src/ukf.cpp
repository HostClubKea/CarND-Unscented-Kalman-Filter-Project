#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>
#include "RadarKalmanFilter.h"
#include "LidarKalmanFilter.h"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() {
    // if this is false, laser measurements will be ignored (except during init)
    use_laser_ = true;

    // if this is false, radar measurements will be ignored (except during init)
    use_radar_ = true;

    if(use_radar_) {
        AbstractKalmanFilter *radarKalmanFilter_ = new RadarKalmanFilter();
        filters_.push_back(radarKalmanFilter_);
    }
    if(use_laser_) {
        AbstractKalmanFilter *lidarKalmanFilter_ = new LidarKalmanFilter();
        filters_.push_back(lidarKalmanFilter_);
    }

}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
    for(AbstractKalmanFilter* filter: filters_){
        filter->ProcessMeasurement(state_, meas_package);
    }
}


