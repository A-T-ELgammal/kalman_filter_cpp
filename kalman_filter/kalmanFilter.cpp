#include <math.h>
#include <iostream>
#include "kalmanFilter.h"
using namespace std;

double Kalmanfilter::getNormalDistributionProbability(gaussianDistribution gd, const double x)
{
    double mu = gd.getMu(),
           sigma = sqrt(gd.getSigmaSquared()),
           normalDistributionProbability = pow(sigma * sqrt(2 * M_PI), -1) * exp(-0.5 * pow((x - mu) / sigma, 2));
    return normalDistributionProbability;
}

// void Kalmanfilter::measurementUpdate(double mu, double sigmaSquared, double muUpdated, double sigmaSquaredUpdated)
// {
// }
