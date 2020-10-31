#pragma once

#include "Minimizer.h"

#include <random>

class RandomMinimizer : public Minimizer
{
public:
    RandomMinimizer(const VectorXd &upperLimit = VectorXd(), const VectorXd &lowerLimit = VectorXd(), double fBest = HUGE_VAL, const VectorXd &xBest = VectorXd())
        : searchDomainMax(upperLimit), searchDomainMin(lowerLimit), fBest(fBest), xBest(xBest){
        fBest = HUGE_VAL;

        // initialize random number generator
        rng.seed(std::random_device()());
        dist = std::uniform_real_distribution<>(0.0,1.0);
    }

    virtual ~RandomMinimizer() {}

    virtual bool minimize(const ObjectiveFunction *function, VectorXd &x) {
        for (int i = 0; i < iterations; ++i) {

            // generate a random number in [0, 1]
            // double a = dist(rng);

            ////////////////////////////////////// 1.1

            // iterate over all dimensions
            for (int j = 0; j < x.size(); j++)
            {
                // generate random candidate x values within sampling domain
                double rand = dist(rng);
                // x does not contain any value, hence fill with random values
                x[j] = searchDomainMin[j] + rand * (searchDomainMax[j] - searchDomainMin[j]);
            }

            // if candidate x performs better than currently best function value
            if (function->evaluate(x) < fBest)
            {
                // store x and function value
                fBest = function->evaluate(x);
                xBest = x;
            }
            
            ////////////////////////////////////// 1.1
        }
        // pass x for visualization
        x = xBest;
        return false;
    }

public:
    int iterations = 1;
    VectorXd searchDomainMax, searchDomainMin;
    VectorXd xBest;
    double fBest;

    std::uniform_real_distribution<double> dist;
    std::mt19937 rng;
};
