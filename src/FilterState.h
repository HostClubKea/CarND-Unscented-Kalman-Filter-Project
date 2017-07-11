#ifndef EXTENDEDKF_FILTERSTATE_H
#define EXTENDEDKF_FILTERSTATE_H
#include "Eigen/Dense"

class FilterState {
public:
    FilterState();

public:
    // initially set to false, set to true in first call of ProcessMeasurement
    bool is_initialized_;

    // previous timestamp
    long previous_timestamp_;

    // state vector
    Eigen::VectorXd x_;

    // state covariance matrix
    Eigen::MatrixXd P_;
};


#endif //EXTENDEDKF_FILTERSTATE_H
