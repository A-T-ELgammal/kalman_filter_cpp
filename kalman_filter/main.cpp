#include <iostream>
#include "kalmanFilter.h"

using namespace std;

int main()
{
    Kalmanfilter kf;
    gaussianDistribution gd1(10, 4);
    double x = 8;

    cout << "probablity is: " << kf.getNormalDistributionProbability(gd1, x) << endl;

    return 0;
}
