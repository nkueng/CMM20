#pragma once

#include <utility>
#include "RigidBodySimulation.h"
#include <RandomMinimizer.h>

class MatchTrajectoryObjective : public ObjectiveFunction
{
public:
    MatchTrajectoryObjective(RigidBodySimulation sim, Matrix<double, -1, 2> targetPath)
        : sim(std::move(sim)), targetTrajectory(std::move(targetPath)) {}

    double evaluate(const VectorXd& p) const override {
        
        sim.setDesignParameters(p);
        Matrix<double, -1, 2> currTrajectory = sim.recordTrajectory();

        return (currTrajectory - targetTrajectory).squaredNorm();
    }

public:
    mutable RigidBodySimulation sim;
    Matrix<double, -1, 2> targetTrajectory;
};

class MechanismOptimizer
{
public:
    MechanismOptimizer() = default;

    MechanismOptimizer(const RigidBodySimulation &sim) : sim(sim) {
        p = sim.getDesignParameters();
    }

    void optimizeTrajectory() {
        MatchTrajectoryObjective obj(sim, targetPath);
        minimizer.searchDomainMin = 0.95 * p;
        minimizer.searchDomainMax = 1.05 * p;
        minimizer.minimize(&obj, p);
    }

public:
    RigidBodySimulation sim;
    Matrix<double, -1, 2> targetPath;

    RandomMinimizer minimizer;
    VectorXd p;

};
