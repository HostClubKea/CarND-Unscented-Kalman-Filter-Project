#ifndef EXTENDEDKF_ABSTRACTKALMANFILTER_H
#define EXTENDEDKF_ABSTRACTKALMANFILTER_H

#include <iostream>
#include "Eigen/Dense"
#include "FilterState.h"
#include "measurement_package.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using namespace std;

class AbstractKalmanFilter {
public:

    // measurement matrix
    Eigen::MatrixXd H_;

    // measurement covariance matrix
    Eigen::MatrixXd R_;

    ///* Weights of sigma points
    VectorXd weights_;

    ///* predicted sigma points matrix
    MatrixXd Xsig_pred_;

    // State dimension
    int n_x_;

    // Augmented state dimension
    int n_aug_;

    // Sigma point spreading parameter
    double lambda_;

    // Process noise standard deviation longitudinal acceleration in m/s^2
    double std_a_;

    // Process noise standard deviation yaw acceleration in rad/s^2
    double std_yawdd_;

    // the current NIS
    double NIS_;

    // total measurements processed by sensor
    int total_measurements;

    // NIS top border
    double NIS_top;

    // NIS bottom border
    double NIS_bot;

    // NIS above top border
    int total_above_top;

    // NIS below bottom border
    int total_above_bot;

    // NIS inside  borders
    int total_inside;


    /**
     * Constructor
     */
    AbstractKalmanFilter();

    /**
     * Destructor
     */
    virtual ~AbstractKalmanFilter();

    void ProcessMeasurement(FilterState &state, const MeasurementPackage &measurement_pack);

    virtual bool IsProcessible(const MeasurementPackage &measurement_pack)  = 0;

protected:
    virtual void Init(FilterState &state, const MeasurementPackage &measurement_pack);

    void Predict(FilterState &state, const float dt);

    virtual void Update(FilterState &state, const VectorXd &z) = 0;

    MatrixXd GenerateSigmaPoints(FilterState &state);

    MatrixXd PredictSigmaPoints(MatrixXd Xsig_aug, double dt);

    VectorXd PredictSigmaPoint(VectorXd sigma_point, double dt);

    VectorXd CalculateTransition(VectorXd sigma_point, double dt);

    VectorXd ComputeMean(MatrixXd sig, int dimensions);

    MatrixXd ComputeCovariance(MatrixXd sig, VectorXd mean, int angle_index, int dimensions);

    void UpdateNisStatistics();
};

#endif //EXTENDEDKF_ABSTRACTKALMANFILTER_H
