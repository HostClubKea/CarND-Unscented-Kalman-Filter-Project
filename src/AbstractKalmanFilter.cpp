#include "AbstractKalmanFilter.h"
#include <iostream>

using Eigen::MatrixXd;
using namespace std;


AbstractKalmanFilter::~AbstractKalmanFilter() {}

AbstractKalmanFilter::AbstractKalmanFilter() {

    // state dimension
    n_x_ = 5;

    // Augmented state dimension
    n_aug_ = 7;

    // Sigma point spreading parameter
    lambda_ = 3 - n_aug_;

    // Process noise standard deviation longitudinal acceleration in m/s^2
    std_a_ = 0.5;

    // Process noise standard deviation yaw acceleration in rad/s^2
    std_yawdd_ = 0.5;

    //create vector for weights
    weights_ = VectorXd(2 * n_aug_ + 1);
    // set weights
    double weight_0 = lambda_ / (lambda_ + n_aug_);
    weights_(0) = weight_0;
    //2n+1 weights
    for (int i = 1; i < 2 * n_aug_ + 1; i++) {
        double weight = 0.5 / (n_aug_ + lambda_);
        weights_(i) = weight;
    }

    eps_ = 0.001;

    //NIS routine initialization

    NIS_top = 7.8;
    NIS_bot = 0.35;

    total_measurements = 0;
    total_above_top = 0;
    total_above_bot = 0;
    total_inside = 0;

}

void AbstractKalmanFilter::ProcessMeasurement(FilterState &state, const MeasurementPackage &measurement_pack) {
    if(!IsProcessible(measurement_pack))
        return;

    if(!state.is_initialized_){
        Init(state, measurement_pack);
        return;
    }

    float dt = (measurement_pack.timestamp_ - state.previous_timestamp_) / 1000000.0;
    if(dt <= 0)
        return;

    state.previous_timestamp_ = measurement_pack.timestamp_;

    Predict(state, dt);
    Update(state, measurement_pack.raw_measurements_);

    UpdateNisStatistics();

    //cout << "x = " << state.x_ << endl;
}

void AbstractKalmanFilter::UpdateNisStatistics() {
    total_measurements++;

    if(NIS_ > NIS_bot)
        total_above_bot++;

    if(NIS_ > NIS_top)
        total_above_top++;

    if(NIS_ <= NIS_top && NIS_ >= NIS_bot)
        total_inside++;

    cout << sensorType_ << " NIS inside = " << 100 * total_inside / total_measurements << "%" << endl;
    cout << sensorType_ << " NIS above bottom = " << 100 * total_above_bot / total_measurements << "%" << endl;
    cout << sensorType_ << " NIS above top = " << 100 * total_above_top / total_measurements << "%" << endl;
    cout << sensorType_ << " NIS = " << NIS_ << endl;
}

void AbstractKalmanFilter::Init(FilterState &state, const MeasurementPackage &measurement_pack) {
    state.previous_timestamp_ = measurement_pack.timestamp_;
    // done initializing, no need to predict or update
    state.is_initialized_ = true;
}

void AbstractKalmanFilter::Predict(FilterState &state, const float dt) {
    // Generate Sigma points
    MatrixXd Xsig_aug = GenerateSigmaPoints(state);

    // Use the prediction function to predict the k+1 values for these sigma points
    Xsig_pred_ = PredictSigmaPoints(Xsig_aug, dt);

     // Use predicted sigma points to compute new mean
    state.x_ << ComputeMean(Xsig_pred_);

    // Use predicted sigma points to compute covariance matrix
    state.P_ << ComputeCovariance(Xsig_pred_, state.x_, 3);
}

MatrixXd AbstractKalmanFilter::GenerateSigmaPoints(FilterState &state) {
    //create sigma point matrix
    MatrixXd Xsig_aug = MatrixXd(n_aug_, 2 * n_aug_ + 1);

    // Create Augmented state mean vector x_aug and augmented state covariance matrix P_aug
    VectorXd x_aug = VectorXd(n_aug_);
    x_aug.head(5) = state.x_;
    x_aug(5) = 0;
    x_aug(6) = 0;

    //create augmented state covariance
    MatrixXd P_aug = MatrixXd(n_aug_, n_aug_);
    P_aug.fill(0.0);
    P_aug.topLeftCorner(n_x_, n_x_) = state.P_;
    P_aug(n_x_, n_x_) = pow(std_a_, 2);
    P_aug(n_x_ + 1, n_x_ + 1) = pow(std_yawdd_, 2);

    //create square root matrix
    MatrixXd A = P_aug.llt().matrixL();
    if (P_aug.llt().info() == Eigen::NumericalIssue) {
        // if decomposition fails, we have numerical issues
        std::cout << "LLT failed!" << std::endl;
        throw std::range_error("LLT failed");
    }

    //create augmented sigma points
    Xsig_aug.col(0) = x_aug;
    MatrixXd term = sqrt(lambda_ + n_aug_) * A;
    for (int i = 0; i < n_aug_; ++i) {
        Xsig_aug.col(i + 1) = x_aug + term.col(i);
        Xsig_aug.col(i + n_aug_ + 1) = x_aug - term.col(i);
    }
    return Xsig_aug;
}

MatrixXd AbstractKalmanFilter::PredictSigmaPoints(MatrixXd Xsig_aug, double dt) {
    MatrixXd predictions(n_x_, 2 * n_aug_ + 1);
    for (int i = 0; i < 2 * n_aug_ + 1; ++i) {
        predictions.col(i) = PredictSigmaPoint(Xsig_aug.col(i), dt);
    }
    return predictions;
}

VectorXd AbstractKalmanFilter::PredictSigmaPoint(VectorXd sigma_point, double dt) {
    return sigma_point.head(n_x_) + CalculateTransition(sigma_point, dt);
}

VectorXd AbstractKalmanFilter::CalculateTransition(VectorXd sigma_point, double dt) {
    VectorXd transition(n_x_);
    transition.fill(0.0);

    double px = sigma_point(0);
    double py = sigma_point(1);
    double v = sigma_point(2);
    double psi = sigma_point(3);
    double psi_dot = sigma_point(4);
    double long_acceleration = sigma_point(5);
    double yaw_rate_acceleration = sigma_point(6);

    if (fabs(psi_dot) > eps_) {
        transition(0) = (v / (float) psi_dot * (sin(psi + psi_dot * dt) - sin(psi)));
        transition(1) = (v / (float) psi_dot * (-cos(psi + psi_dot * dt) + cos(psi)));
        transition(2) = 0;
        transition(3) = (psi_dot * dt);
        transition(4) = 0;
    } else {
        transition(0) = (v * cos(psi) * dt);
        transition(1) = (v * sin(psi) * dt);
        transition(2) = 0;
        transition(3) = 0;
        transition(4) = 0;
    }

    VectorXd process_noise(n_x_);
    process_noise(0) = (1 / 2.0 * dt * dt * cos(psi) * long_acceleration);
    process_noise(1) = (1 / 2.0 * dt * dt * sin(psi) * long_acceleration);
    process_noise(2) = (long_acceleration * dt);
    process_noise(3) = (1 / 2.0 * dt * dt * yaw_rate_acceleration);
    process_noise(4) = (yaw_rate_acceleration * dt);


    return transition + process_noise;
}

VectorXd AbstractKalmanFilter::ComputeMean(MatrixXd sigma_points) {
    //Calculate mean from sigma points
    VectorXd mean = VectorXd(sigma_points.rows());
    //predicted state mean
    mean.fill(0.0);
    for (int i = 0; i < 2 * n_aug_ + 1; i++) { //iterate over sigma points
        mean = mean + weights_(i) * sigma_points.col(i);
    }

    return mean;
}

MatrixXd AbstractKalmanFilter::ComputeCovariance(MatrixXd sigma_points, VectorXd mean, int angle_index) {
    //Calculate covariance matrix from sigma points
MatrixXd P = MatrixXd(sigma_points.rows(), sigma_points.rows());

    P.fill(0.0);
    for (int i = 0; i < 2 * n_aug_ + 1; i++) { //iterate over sigma points
        // state difference
        VectorXd diff = sigma_points.col(i) - mean;
        //angle normalization
        if(angle_index >= 0) {
            diff(angle_index) = NormalizeAngle(diff(angle_index));
        }

        P = P + weights_(i) * diff * diff.transpose();
    }
    return P;
}

double AbstractKalmanFilter::NormalizeAngle(double angle) const {
    while (angle > M_PI) angle -= 2. * M_PI;
    while (angle < -M_PI) angle += 2. * M_PI;
    return angle;
}
