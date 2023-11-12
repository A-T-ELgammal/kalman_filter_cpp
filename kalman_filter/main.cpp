#include <iostream>
#include "kalmanFilter.h"

using namespace std;

int main()
{
    // test the kalman filter methods

    Kalmanfilter kf;
    gaussianDistribution gd1(10, 4);
    double x = 8;
    gaussianDistribution prior(20, 9);
    gaussianDistribution measurement(30, 3);
    ////////////////////////////////////////////////////
    // test ND-probability
    //  cout << "probablity is: " << kf.getNormalDistributionProbability(gd1, x) << endl;
    ////////////////////////////////////////////////////
    // test update measurements
    gaussianDistribution updated = kf.measurementUpdate(prior, measurement);
    cout << "the updated mu is: " << updated.getMu() << "the updated sigma is : " << updated.getSigmaSquared() << endl;
}
