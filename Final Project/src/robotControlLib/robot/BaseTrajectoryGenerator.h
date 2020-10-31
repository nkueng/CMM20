#pragma once

#include <utils/utils.h>
#include <optLib/ConstrainedObjectiveFunction.h>
    namespace BaseTrajectoryGenerator {
            struct BaseTrajectoryGeneratorMessage {
                double referenceBaseZVelocity;
                double referenceYawRate;
                bool trajectoryHasComeToEnd;
            };
        // setting up custom objective class for SQPMinimizer adjusting virtual methods to our use-case in RobotControl
        class BaseTrajectoryGenerator {
        public:
        // constructor
        BaseTrajectoryGenerator(Robot* robot) : 
            robot(robot),
            initialBasePosition(dVector::Zero(2)),
            initialBaseYaw(0.),
            trajectoryIterator(0),
            base(robot->getRigidBody(0)),
            isInit(true) {}

        // call base trajectory generator
        BaseTrajectoryGeneratorMessage generateBaseTrajectory(const dVector & targetBasePosition, const double & targetBaseYaw) {
            if(isInit) {
                const P3D basePosition = base->state.pos;
                initialBasePosition[0] = basePosition.x;
                initialBasePosition[1] = basePosition.z;

                //const Quaternion orientation = base->state.orientation;
                V3D a = V3D();
                V3D b = V3D();
                V3D c = V3D(); 
                double alpha = 0.;
                double beta = 0.; 
                double gamma = 0.;
                //computeEulerAnglesFromQuaternion(orientation, a, b, c, alpha, beta, gamma);
                initialBaseYaw = 0.;

                this->targetBaseYaw = targetBaseYaw;
                this->targetBasePosition = targetBasePosition;

                calculateTrajectory();
				numbersOfIterationsTillTrajectoryComesToEnd *= 120;
                // DEBUG
                std::cout << "Reference velocity = " << referenceBaseZVelocity << std::endl;
                std::cout << "Reference yaw rate = " << referenceYawRate << std::endl;
				std::cout << "Number of iterations = " << numbersOfIterationsTillTrajectoryComesToEnd << std::endl;
                isInit = false;
                }

            BaseTrajectoryGeneratorMessage msg;

            if (trajectoryIterator < numbersOfIterationsTillTrajectoryComesToEnd) {
                msg.referenceBaseZVelocity = referenceBaseZVelocity;
                msg.referenceYawRate = referenceYawRate;
                msg.trajectoryHasComeToEnd = false;
				std::cout << "Iteration of trajectory = " << trajectoryIterator << std::endl;
                trajectoryIterator++;
            }
            else {
                msg.referenceBaseZVelocity = 0.;
                msg.referenceYawRate = 0.;
                msg.trajectoryHasComeToEnd = true;
            }

            return msg;
        }

        void calculateTrajectory() {
           // Figure out whether target poisition is feasible to achieve
           const double differenceInBaseXPosition = - initialBasePosition[0] + targetBasePosition[0];
           const double differenceInBaseZPosition = - initialBasePosition[1] + targetBasePosition[1];
           const double differenceInBaseYaw = targetBaseYaw - initialBaseYaw;
           const double signOfBaseVelocity = differenceInBaseZPosition / std::abs(differenceInBaseZPosition);
           const double signOfYawRate = differenceInBaseXPosition / std::abs(differenceInBaseXPosition);
           double candidateReferenceYawRate = 0.;
           double candidateReferenceBaseZVelocity = 0.;
           double numbersOfIterationsTillTrajectoryComesToEndApplyingMaxRate = 
                int(std::round(std::abs(differenceInBaseYaw) / (maxYawRate * minimalFinalTime)));

           double baseVelocityToYawRateRatio1 = (differenceInBaseXPosition) / (1. - std::cos(differenceInBaseYaw));
           double baseVelocityToYawRateRatio2 = (differenceInBaseZPosition) / (std::sin(differenceInBaseYaw));
           // If this two ratios are equal, then 
           if (almostEqual(baseVelocityToYawRateRatio1, baseVelocityToYawRateRatio2)) {
                // DEBUG
                std::cout << "Disregarding the constraints the reference position is feasible." << std::endl;
                // Look for minimal final time yaw rate.
                candidateReferenceYawRate = signOfYawRate * maxYawRate;
                // Based on maximal yaw rate calculate candidate base velocity.
                candidateReferenceBaseZVelocity = candidateReferenceYawRate * baseVelocityToYawRateRatio1;
                // Check whether the candidate velocity is whitin the constraints.
                if ((candidateReferenceBaseZVelocity >= - maxBaseZVelocityWRTBase) 
                     && (candidateReferenceBaseZVelocity <= maxBaseZVelocityWRTBase)) {
                        std::cout << "Reference position is feasible within minimal time." << std::endl;
                        referenceYawRate = candidateReferenceYawRate;
                        referenceBaseZVelocity = candidateReferenceBaseZVelocity; 
                        numbersOfIterationsTillTrajectoryComesToEnd = numbersOfIterationsTillTrajectoryComesToEndApplyingMaxRate;
                        return;
                }

                // Calculate candidate reference yaw based on maximal velocity
                candidateReferenceBaseZVelocity = signOfBaseVelocity * maxBaseZVelocityWRTBase;
                candidateReferenceYawRate = candidateReferenceBaseZVelocity / baseVelocityToYawRateRatio1;

                // Check whether the candidate velocity is whitin the constraints.
                if ((candidateReferenceYawRate >= - maxYawRate) 
                     && (candidateReferenceYawRate <= maxYawRate)) {
                        std::cout << "Reference position is feasible applying maximal velocity." << std::endl;
                        referenceYawRate = candidateReferenceYawRate;
                        referenceBaseZVelocity = candidateReferenceBaseZVelocity; 
                        numbersOfIterationsTillTrajectoryComesToEnd = std::round(differenceInBaseYaw / referenceYawRate);
                        return;
                }

                /*
                // We arrive here if the max yaw rate leads to velocity out of bounds. We try to find out the smallest
                for (int i = numbersOfIterationsTillTrajectoryComesToEndApplyingMaxRate + 1; 
                     i < numbersOfIterationsTillTrajectoryComesToEndApplyingMaxRate + offsetToNumbersOfIterationsTillTrajectoryComesToEndApplyingMaxRate;
                     i++) {
                    candidateReferenceYawRate = signOfYawRate * std::abs(differenceInBaseYaw) / (double(n) * minimalFinalTime);
                    candidateReferenceBaseZVelocity = signOfBaseVelocity * candidateReferenceYawRate * baseVelocityToYawRateRatio1;
                    if ((candidateReferenceBaseZVelocity >= - maxBaseZVelocityWRTBase) 
                     && (candidateReferenceBaseZVelocity <= maxBaseZVelocityWRTBase)) {
                        std::cout << "Reference position is feasible but not opitmal in time due to velocity constraint." << std::endl;
                        referenceYawRate = candidateReferenceYawRate;
                        referenceBaseZVelocity = candidateReferenceBaseZVelocity; 
                        numbersOfIterationsTillTrajectoryComesToEnd = i;
                        return; 
                     }
                }
                */

                // If we arrive here we give up to 
           }

           int bestNumbersOfIterationsTillTrajectoryComesToEnd = 0;
           double smallestFucntionValueSoFar = std::numeric_limits<double>::infinity();

           if ((candidateReferenceBaseZVelocity >= - maxBaseZVelocityWRTBase) 
                     && (candidateReferenceBaseZVelocity <= maxBaseZVelocityWRTBase)) {
                // We arrive here if the max yaw rate leads to velocity out of bounds. We try to find out the smallest
                for (int i = numbersOfIterationsTillTrajectoryComesToEndApplyingMaxRate; 
                        i < numbersOfIterationsTillTrajectoryComesToEndApplyingMaxRate + offsetToNumbersOfIterationsTillTrajectoryComesToEndApplyingMaxRate;
                        i++) {
                    candidateReferenceYawRate = signOfYawRate * std::abs(differenceInBaseYaw) / (double(i) * minimalFinalTime);
                    std::cout << "yaw = " << candidateReferenceYawRate << std::endl;
                    double numeratorOfCandidateReferenceBaseZVelocity = - candidateReferenceYawRate * ((- differenceInBaseXPosition) 
                                                                    * (1. - std::cos(differenceInBaseYaw)) + (- differenceInBaseZPosition) 
                                                                    * (std::sin(differenceInBaseYaw)));
                    double denominatorOfCandidateReferenceBaseZVelocity = 2. * (1. - cos(differenceInBaseYaw));
                    candidateReferenceBaseZVelocity = numeratorOfCandidateReferenceBaseZVelocity / denominatorOfCandidateReferenceBaseZVelocity;
                    std::cout << "velocity = " << candidateReferenceBaseZVelocity << std::endl;
                    if ((candidateReferenceBaseZVelocity >= - maxBaseZVelocityWRTBase) 
                        && (candidateReferenceBaseZVelocity <= maxBaseZVelocityWRTBase)) {
                        double functionValuePart1 = candidateReferenceBaseZVelocity / candidateReferenceYawRate * (1. - cos(differenceInBaseYaw)) 
                                                - differenceInBaseXPosition;
                        double functionValuePart2 = candidateReferenceBaseZVelocity / candidateReferenceYawRate * (sin(differenceInBaseYaw)) 
                                                - differenceInBaseZPosition;  
                        double functionValue = functionValuePart1 * functionValuePart1 + functionValuePart2 * functionValuePart2;
                        std::cout << "function value = " << functionValue << std::endl;  

                        if (functionValue < smallestFucntionValueSoFar) {
                            smallestFucntionValueSoFar = functionValue;
                            bestNumbersOfIterationsTillTrajectoryComesToEnd = i;
							std::cout << "Best number of iters = " << bestNumbersOfIterationsTillTrajectoryComesToEnd << std::endl;
                            referenceBaseZVelocity = candidateReferenceBaseZVelocity;
                            referenceYawRate = candidateReferenceYawRate;
                    }                     
                }
					numbersOfIterationsTillTrajectoryComesToEnd = bestNumbersOfIterationsTillTrajectoryComesToEnd;

            }

           }
        }

        bool almostEqual (const double & a, const double & b) {
            const double diff = a - b;
            return (diff < EPSILON) && (-diff < EPSILON);
        }

        // reference to solver to pass matrices for constraints
        Robot* robot;
        RobotRB* base;
        dVector initialBasePosition;
        dVector targetBasePosition;
        double targetBaseYaw;
        double initialBaseYaw;
        const double maxBaseZVelocityWRTBase = 0.045; // TODO 
        const double maxYawRate = 0.125; // TODO
        const double minimalFinalTime= 2.; // TODO optim
        double referenceBaseZVelocity;
        const int offsetToNumbersOfIterationsTillTrajectoryComesToEndApplyingMaxRate = 100;
        double referenceYawRate;
        int trajectoryIterator;
        int numbersOfIterationsTillTrajectoryComesToEnd;
        const int numberOfOptimizedVariables = 2;
        const double EPSILON = 1e-3;

        // member objects for intermediate results passed as references
        dVector Ap;
        dVector Cp;

        // member objects for evaluation of objective generated by RobotControl class
        Matrix A_obj;
        dVector b_obj;

        bool isInit;
        };	
    }