#include "FilterState.h"
using Eigen::VectorXd;
using Eigen::MatrixXd;

FilterState::FilterState() {
    // initial state vector
    x_ = VectorXd(5);

    x_.fill(0.0);

    // initial covariance matrix
    P_ = MatrixXd(5, 5);

    P_ << 1, 0, 0, 0,0,
            0, 1, 0, 0,0,
            0, 0, 1, 0,0,
            0, 0, 0, 1,0,
            0, 0, 0, 0,1;

    ///* initially set to false, set to true in first call of ProcessMeasurement
    is_initialized_ = false;

    previous_timestamp_ = 0;
}
