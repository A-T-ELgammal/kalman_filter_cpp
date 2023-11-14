#include <iostream>
#include "kalmanFilter.h"

using namespace std;

int main()
{
    // test the kalman filter methods

    Kalmanfilter kf;
    ////////////////////////////////////////////////////
    // test ND-probability
    // gaussianDistribution gd1(10, 4);
    // double x = 8;
    //  cout << "probablity is: " << kf.getNormalDistributionProbability(gd1, x) << endl;
    ////////////////////////////////////////////////////
    // test update measurements
    // gaussianDistribution prior(20, 9);
    // gaussianDistribution measurement(30, 3);
    // gaussianDistribution updated = kf.measurementUpdate(prior, measurement);
    // cout << "the updated mu is: " << updated.getMu() << "the updated sigma is : " << updated.getSigmaSquared() << endl;
    // /////////////////////////////////////////////////
    // Measurements and measurement variance
    double measurements[5] = {5, 6, 7, 9, 10};
    double measurement_sig = 4;
    // Motions and motion variance
    double motion[5] = {1, 1, 2, 1, 1};
    double motion_sig = 2;

    // Initial state
    double mu = 0;
    double sig = 1000;

    kf.oneDKalmanFilter(motion, motion_sig, measurements[], measurement_sig, 5, mu, sig);
}
