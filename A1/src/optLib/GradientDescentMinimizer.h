#pragma once

#include "ObjectiveFunction.h"
#include "Minimizer.h"

class GradientDescentFixedStep : public Minimizer {
public:
    GradientDescentFixedStep(int maxIterations=1, double solveResidual=1e-5)
        : maxIterations(maxIterations), solveResidual(solveResidual) {
    }

    int getLastIterations() { return lastIterations; }

    virtual bool minimize(const ObjectiveFunction *function, VectorXd &x) {

        bool optimizationConverged = false;

        // dx is used as gradient here
        VectorXd dx(x.size());

        int i=0;
        for(; i < maxIterations; i++) {
            dx.setZero();
            computeSearchDirection(function, x, dx);

            if (dx.norm() < solveResidual){
                optimizationConverged = true;
                break;
            }

            step(function, dx, x);
        }

        lastIterations = i;

        return optimizationConverged;
    }

protected:
    virtual void computeSearchDirection(const ObjectiveFunction *function, const VectorXd &x, VectorXd& dx) {
        function->addGradientTo(x, dx);
    }

    // Given the objective `function` and the search direction/gradient `dx`, update the candidate `x`
    virtual void step(const ObjectiveFunction *function, const VectorXd& dx, VectorXd& x)
    {
        //////////////////// 1.2

        x = x - dx * stepSize;

        //////////////////// 1.2
    }

public:
    double solveResidual = 1e-5;
    int maxIterations = 1;
    double stepSize = .1;

    // some stats about the last time `minimize` was called
    int lastIterations = -1;
};


class GradientDescentLineSearch : public GradientDescentFixedStep {
public:
    GradientDescentLineSearch(int maxIterations=1, double solveResidual=1e-5, int maxLineSearchIterations=15)
        : GradientDescentFixedStep (maxIterations, solveResidual), maxLineSearchIterations(maxLineSearchIterations){
    }

protected:
    virtual void step(const ObjectiveFunction *function, const VectorXd& dx, VectorXd& x)
    {
        //////////////////// 1.2

        // allocate temporary x update value
        VectorXd x_temp(x.size());
        double temp_stepSize = stepSize;
        x_temp = x - dx * temp_stepSize;

        // only update x if function value f(x_temp) at new location x_temp is smaller
        int counter = 0;
        while (function->evaluate(x_temp) > function->evaluate(x))
        {
            std::cout << "f(x_temp) > f(x), temp_stepSize: " << temp_stepSize << std::endl;
            // half stepsize until objective is satisfied
            temp_stepSize /= 2.;
            x_temp = x - dx * temp_stepSize;
            counter++;

            // exit condition: try maximum maxLineSearchIterations times
            if (counter == maxLineSearchIterations)
            {
                std::cout << "reached max number of iterations in line search" << std::endl;
                break;
            }
        }
        x = x_temp;

        //////////////////// 1.2
    }

protected:
    int maxLineSearchIterations = 15;
};
