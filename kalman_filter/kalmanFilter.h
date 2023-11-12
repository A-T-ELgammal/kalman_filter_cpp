#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

#include <iostream>

using namespace std;

struct gaussianDistribution
{
    gaussianDistribution() {}
    gaussianDistribution(double mu, double sigmaSquared)
    {
        if (mu <= 0 || sigmaSquared <= 0)
            throw invalid_argument("mu or sigmaSquared invalid");

        this->mu = mu;
        this->sigmaSquared = sigmaSquared;
    }

    double getMu() { return mu; }
    double getSigmaSquared() { return sigmaSquared; }

private:
    double mu;
    double sigmaSquared;
};

class Kalmanfilter
{
public:
    double getNormalDistributionProbability(gaussianDistribution gd, const double x);
    gaussianDistribution measurementUpdate(gaussianDistribution gd1, gaussianDistribution gd2);
};

#endif
