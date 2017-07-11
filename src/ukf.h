#ifndef UKF_H
#define UKF_H

#include "measurement_package.h"
#include "Eigen/Dense"
#include <vector>
#include <string>
#include <fstream>
#include <list>
#include "FilterState.h"
#include "AbstractKalmanFilter.h"

using namespace std;

class UKF {
public:
    // if this is false, laser measurements will be ignored (except for init)
    bool use_laser_;

    // if this is false, radar measurements will be ignored (except for init)
    bool use_radar_;

    // Kalman Filter update and prediction math lives in here.
    FilterState state_;

    //Kalman Filter update and prediction math lives in here.
    list<AbstractKalmanFilter*> filters_;

    /**
     * Constructor
     */
    UKF();

    /**
     * Destructor
     */
    virtual ~UKF();

    /**
     * ProcessMeasurement
     * @param meas_package The latest measurement data of either radar or laser
     */
    void ProcessMeasurement(MeasurementPackage meas_package);


};

#endif /* UKF_H */
