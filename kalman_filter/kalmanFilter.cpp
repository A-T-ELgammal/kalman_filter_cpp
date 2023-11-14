#include <math.h>
#include <iostream>
#include "kalmanFilter.h"
using namespace std;

double Kalmanfilter::getNormalDistributionProbability(gaussianDistribution gd, const double x)
{
    double mu = gd.getMu(),
           sigma = sqrt(gd.getSigmaSquared());
    return (pow(sigma * sqrt(2 * M_PI), -1) * exp(-0.5 * pow((x - mu) / sigma, 2)));
}

gaussianDistribution Kalmanfilter::measurementUpdate(gaussianDistribution priorBlief, gaussianDistribution measurement)
{
    double updatedMu, updatedSigmaSquared;
    updatedMu = ((measurement.getSigmaSquared() * priorBlief.getMu()) + (priorBlief.getSigmaSquared() * measurement.getMu())) /
                (priorBlief.getSigmaSquared() + measurement.getSigmaSquared());
    updatedSigmaSquared = pow((pow(priorBlief.getSigmaSquared(), -1)) + (pow(measurement.getSigmaSquared(), -1)), -1);
    return {updatedMu, updatedSigmaSquared};
}

gaussianDistribution Kalmanfilter::statePrediction(gaussianDistribution posterior, gaussianDistribution motion)
{
    double newMu = posterior.getMu() + motion.getMu();
    double newSigmaSquared = posterior.getSigmaSquared() + motion.getSigmaSquared();
    return {newMu, newSigmaSquared};
}

void Kalmanfilter::oneDKalmanFilter(double motion[], double &motionSigmaSquared, const double measurements[], const double &measurementsSigmaSquared, int totalMotions, double initMu, double initSigmaSquared)
{
    for (int i = 0; i < totalMotions; i++)
    {

        gaussianDistribution initialStates{initMu, initSigmaSquared};

        gaussianDistribution updated = measurementUpdate(initialStates, gaussianDistribution{measurements[i], measurementsSigmaSquared});
        gaussianDistribution newEstimate = statePrediction(updated, {motion[i], motionSigmaSquared});

        initMu = newEstimate.getMu();
        initSigmaSquared = newEstimate.getSigmaSquared();
        cout << "updated state: mu- " << initMu << "sigmaSquared- " << initSigmaSquared << endl;
    }
}
