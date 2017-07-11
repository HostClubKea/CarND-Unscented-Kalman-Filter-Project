#ifndef EXTENDEDKF_RADARKALMANFILTER_H
#define EXTENDEDKF_RADARKALMANFILTER_H

#include "AbstractKalmanFilter.h"
#include "tools.h"

class RadarKalmanFilter : public AbstractKalmanFilter {
public:
    double eps;

    // Measurement dimension
    int n_z_;

    // Radar measurement noise standard deviation radius in m
    double std_radr_;

    // Radar measurement noise standard deviation angle in rad
    double std_radphi_;

    // Radar measurement noise standard deviation radius change in m/s
    double std_radrd_;

    RadarKalmanFilter();

    virtual ~RadarKalmanFilter();

    bool IsProcessible(const MeasurementPackage &measurement_pack) override ;

protected:
    void Init(FilterState &state, const MeasurementPackage &measurement_pack) override;

    void Update(FilterState &state, const VectorXd &z) override;

private:

    MatrixXd TransformSigmaPointsToMeasurementSpace() const;

    MatrixXd CalculateCrossCorrelation(const FilterState &state, MatrixXd &Zsig, const VectorXd &z_pred) const;
};


#endif //EXTENDEDKF_RADARKALMANFILTER_H
