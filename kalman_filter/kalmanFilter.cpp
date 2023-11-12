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

gaussianDistribution Kalmanfilter::measurementUpdate(gaussianDistribution priorBlief, gaussianDistribution measurement)
{
    double updatedMu, updatedSigmaSquared;
    updatedMu = ((measurement.getSigmaSquared() * priorBlief.getMu()) + (priorBlief.getSigmaSquared() * measurement.getMu())) /
                (priorBlief.getSigmaSquared() + measurement.getSigmaSquared());
    updatedSigmaSquared = pow((pow(priorBlief.getSigmaSquared(), -1)) + (pow(measurement.getSigmaSquared(), -1)), -1);
    gaussianDistribution postreior(updatedMu, updatedSigmaSquared);
    return postreior;
}