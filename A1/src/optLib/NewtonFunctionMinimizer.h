    #pragma once

#include "ObjectiveFunction.h"
#include "GradientDescentMinimizer.h"

class NewtonFunctionMinimizer : public GradientDescentLineSearch {
public:
    NewtonFunctionMinimizer(int maxIterations = 1, double solveResidual = 0.0001, int maxLineSearchIterations = 15)
        : GradientDescentLineSearch(maxIterations, solveResidual, maxLineSearchIterations) {	}

    virtual ~NewtonFunctionMinimizer() {}

protected:
    virtual void computeSearchDirection(const ObjectiveFunction *function, const VectorXd &x, VectorXd& dx) {

        //////////////////// 1.3

        // solve linear system 
        // A*x = b
        // hessian * dx = -grad (careful: while this expression minimizes the taylor approx of the objective 
        // function, we are actually interested in the dx with reverse direction (i.e., in direction of gradient) to use it for line search.
        // Hence the missing minus sign below.)
        Eigen::SimplicialLDLT<SparseMatrixd, Eigen::Lower> solver;
        function->getHessian(x, hessian);
        // regularize hessian
        Eigen::MatrixXd eye = Eigen::Matrix2d::Identity(x.size(), x.size());

        hessian = hessian + reg * eye;
        solver.compute(hessian);
        VectorXd grad;
        grad = function->getGradient(x);
        dx = solver.solve(grad);

        //////////////////// 1.3
    }

public:
    SparseMatrixd hessian;
    std::vector<Triplet<double>> hessianEntries;
    double reg = 1.0;
};
