#pragma once

#include <robot/Robot.h>
#include <robot/GeneralizedCoordinatesRobotRepresentation.h>
#include <utils/utils.h>
#include <chrono>
#include <ctime>

namespace MotionGenerator{
    struct MotionGeneratorMessage {
        P3D desiredBasePosition;
        V3D desiredBaseVelocity;
        V3D desiredBaseOrient;
        V3D desiredTurningRate;
        std::vector<P3D> desiredFootPositions;
        std::vector<P3D> desiredFootVelocities;
        std::vector<P3D> desiredFootAccelerations;
        bool * isInContact;
    };

    class MotionGenerator {
        public:
            MotionGenerator(Robot* robot, const double & dt) : 
                robot(robot),
                dt(dt),
                numberOfTimeStampsToNextPhase((phaseDuration * desiredGatePeroid) / dt),
                base(robot->getRigidBody(0)),
                currentPhaseIterator(0),
                strideIterator(0),
                desiredYaw(0.),
                desiredYawRate(0.),
                desiredBasePositionNow(P3D()),
                periodIterator(0),
                currentStride(0.),
                strideIteratorUpdated(false),
                desiredBaseVelocity(0., 0., 0.),
                isInit(true) {
                // Fill foot rigid body vector with rigid body pointers for each foot.
                footRBs.clear();
                for(int i = footRBsStartingIndex; i < footRBsStartingIndex + numberOfLegs; i++) {
                    footRBs.push_back(robot->getRigidBody(i));
                    previousdesiredFootYAccerlations.push_back(0.);
                }
            }

            MotionGeneratorMessage generateMotion(const double & desiredZVelocity, const double & desiredXVelocity, const double & desiredYawRate, bool * gaitMode, bool * footenable) {
                updateCurrentStride();

                // Main logic
                if(currentStride == 0.) {
                    desiredBaseVelocity[0] = desiredXVelocity;
                    desiredBaseVelocity[1] = 0.;
                    desiredBaseVelocity[2] = desiredZVelocity;
                    this->desiredYawRate = desiredYawRate;

                    for (int i = 0; i < numberOfLegs; i++) {
                        for (int j = 0; j < 6; j++) {
                            gaitPattern[i][j] = gaitMode[i * 6 + j];
                        }
                    }
        
                    for (int j = 0; j < numberOfLegs; j++) {
                        numberOfSwingsInPeriod[j] = 0;
                        for (int i = 0; i < numberOfLegs; i++) {
                            if (!gaitPattern[j][i]) numberOfSwingsInPeriod[j] += 1;
                        }
                    }

					for (int i = 0; i < numberOfLegs; i++)
					{
						footEnabled[i] = footenable[i];
					}

                    prepareNextPhase();

                    if(isInit) isInit = false;
                }
                // Condition where the period comes to its end.
                else if(currentStride >= 1.) {
                    forceContact();
                    if(contactChecked()) {
                        // DEBUG
                        std::cout << "Period done." << std::endl;
                        currentStride = 0.; 
                        currentPhaseIterator = 0;
                        strideIterator = 0;
                        periodIterator = 0;
                        periodcounter++;
                    }
                }
                // Condition where we should switch swing legs.
                else if(currentStride >= double(currentPhaseIterator + 1.) * phaseDuration) {
                    forceContact();
                    periodIterator = 0;

					if (!strideIteratorUpdated) {
						strideIterator++;
						strideIteratorUpdated = true;
					}
        
                    if(contactChecked()) {
                        // DEBUG
                        std::cout << "Phase done." << std::endl;
                        periodIterator = 1;
                        desiredBaseVelocity[2] = desiredZVelocity;
                        desiredBaseVelocity[0] = desiredXVelocity;
                        desiredBaseVelocity[1] = 0.;
                        this->desiredYawRate = desiredYawRate;
                        currentPhaseIterator++; 
                        prepareNextPhase();
                        strideIteratorUpdated = false;
                    }
 
                }
                // Regular running condition.
                else {
                    interpolateFootTrajectories();
                    computeDesiredBasePosition(stanceWeight, swingWeight);
                }

                desiredYaw += this->desiredYawRate * dt;
                Matrix rotationMatrix(3, 3);
                rotationMatrix <<  std::cos(desiredYaw),  0., std::sin(desiredYaw),
                                                     0.,  1.,                   0.,
                                  -std::sin(desiredYaw),  0., std::cos(desiredYaw);
                auto rotatedBaseVelocity = rotationMatrix * desiredBaseVelocity; 

                // DEBUG
                // std::cout << "Current stride = " << currentStride << std::endl;
                std::cout << "Current phase = " << currentPhaseIterator << std::endl;
                std::cout << "Current period = " << periodcounter << std::endl;
                // std::cout << "Target z position of right front foot = " << desiredFootPositions[0].z << std::endl;
                // std::cout << "Target x position of right front foot = " << desiredFootPositions[0].x << std::endl;
                // std::cout << "Target height of right front foot = " << desiredFootPositions[0].y << std::endl;
                // std::cout << "Target z velocity of right front foot = " << desiredFootVelocities[0].z << std::endl;
                // std::cout << "Target x velocity of right front foot = " << desiredFootVelocities[0].x << std::endl;
                // std::cout << "Target vertical velocity of right front foot = " << desiredFootVelocities[0].y << std::endl;
                // std::cout << "Target z acceleration of right front foot = " << desiredFootAccelerations[0].z << std::endl;
                // std::cout << "Target x acceleration of right front foot = " << desiredFootAccelerations[0].x << std::endl;
                // std::cout << "Target vertical acceleration of right front foot = " << desiredFootAccelerations[0].y << std::endl;
                // std::cout << "Target z position of left front foot = " << desiredFootPositions[1].z << std::endl;
                // std::cout << "Target x position of left front foot = " << desiredFootPositions[1].x << std::endl;
                // std::cout << "Target height of left front foot = " << desiredFootPositions[1].y << std::endl;
                // std::cout << "Target z velocity of left front foot = " << desiredFootVelocities[1].z << std::endl;
                // std::cout << "Target x velocity of left front foot = " << desiredFootVelocities[1].x << std::endl;
                // std::cout << "Target vertical velocity of left front foot = " << desiredFootVelocities[1].y << std::endl;
                // std::cout << "Target z acceleration of left front foot = " << desiredFootAccelerations[1].z << std::endl;
                // std::cout << "Target x acceleration of left front foot = " << desiredFootAccelerations[1].x << std::endl;
                // std::cout << "Target vertical acceleration of left front foot = " << desiredFootAccelerations[1].y << std::endl;
                // std::cout << "Desired yaw = " << desiredYaw << std::endl;
                // std::cout << "Yaw rate = " << this->desiredYawRate << std::endl;
                // std::cout << "Target z velocity of base = " << rotatedBaseVelocity[2] << std::endl;
                // std::cout << "Target x velocity of base = " << rotatedBaseVelocity[0] << std::endl;

                MotionGeneratorMessage msg;
                msg.desiredBasePosition = desiredBasePosition;         
                msg.desiredBaseVelocity = rotatedBaseVelocity;
                msg.desiredBaseOrient[1] = desiredYaw;
                msg.desiredTurningRate[1] = this->desiredYawRate;
                msg.desiredFootPositions = desiredFootPositions;
                msg.desiredFootVelocities = desiredFootVelocities;
                msg.desiredFootAccelerations = desiredFootAccelerations;
                msg.isInContact = isInContact;

                return msg;
            }

        // private:
            void prepareNextPhase() {
                    updateFootModes();
                    updateBasePositionAndVelocity();
                    updateFootPositions();
                    updateTargetFootholdPosition();
                    updateSagittalAndCoronalTrajectoryCoefficients();
            }

            void updateFootPositions() {
                footPositions.clear();
                previousTargetFootholdPositions.clear();

                for(const auto foot : footRBs) {
                    auto footPositionWRTfoot = foot->rbProps.endEffectorPoints[0];
                    auto footPositionWRTW = foot->state.getWorldCoordinates(footPositionWRTfoot);
                    // DEBUG
                    // std::cout << "Foot position = " << V3D(footPositionWRTW) << std::endl;
                    footPositions.push_back(footPositionWRTW);

                    // For planning the target footholds we apply the previous target foothold position to be
                    // the inital point of trajectory. This is done to prevent jumps in the reference trajectory
                    // which causes in genreal problems for controllers. At initialization of a the gate controller
                    // we have to intialize the previous target foothold position to be the current actual position.
                    // Here we read that initial position into member.
                    if(isInit) {
                        // Safety
                        footPositionWRTW.y = 0.;
                        previousTargetFootholdPositions.push_back(footPositionWRTW);

                        auto projectedBasePosition = basePosition;
                        projectedBasePosition.y = 0.;
                        auto differenceVector = footPositionWRTW - projectedBasePosition;
                        initialHorizontalDistanceVectorFromBaseToFootPositions.push_back(differenceVector);
                    }
                }
            }

            void updateBasePositionAndVelocity() {
                basePosition = base->state.pos;
                if(isInit) referenceBaseHeight = basePosition.y;
                // DEBUG
                // std::cout << "Base Height = " << basePosition.y << std::endl;
                baseVelocity = base->state.velocity;
                // std::cout << "Base velocity x = " << baseVelocity[0] << std::endl;
                // std::cout << "Base velocity y = " << baseVelocity[1] << std::endl;
                // std::cout << "Base velocity z = " << baseVelocity[2] << std::endl;
            }

            void updateTargetFootholdPosition() {
                if(!isInit) {
                    // For planning the target footholds we apply the previous target foothold position to be
                    // the inital point of trajectory. This is done to prevent jumps in the reference trajectory
                    // which causes in genreal problems for controllers. Under regular running conditions (everthing
                    // but initialization) we overwrite the previoud target position with the last target position here.
                    previousTargetFootholdPositions.clear();
                    previousTargetFootholdPositions = targetFootholdPositions;
                }

                targetFootholdPositions.clear();
                desiredFootPositions.clear();
                desiredFootVelocities.clear();
                desiredFootAccelerations.clear();

                bool calculateDesiredBasePositionAtNextPhase = true;
                auto desiredBasePositionAtNextPhase = desiredBasePositionNow;
                for(int i = 0; i < numberOfLegs; i++) {
                    // DEBUG
                    // std::cout << "i = " << i << std::endl;
                if (footEnabled[i])
                {
                  if (!isInContact[i]) {
                    int numberOfStepsToNextSwing = int((desiredGatePeroid / double(numberOfSwingsInPeriod[i])) / dt);
                    // DEBUG
                    //std::cout << "numberOfStepsToNextSwing = " << numberOfStepsToNextSwing << std::endl;
                    double timeNow = double(currentPhaseIterator) * phaseDuration * desiredGatePeroid;
                    // DEBUG
                    // std::cout << "timeNow = " << timeNow << std::endl;
                    desiredYawAtTimeStamp = desiredYaw; //timeNow * desiredYawRate;
                    // DEBUG
                    // std::cout << "desiredYawAtTimeStamp = " << desiredYawAtTimeStamp << std::endl;

                    auto desiredBasePositionAtNextSwingState = desiredBasePositionNow;
                    Matrix orientationNextSwingState(3, 3);
                    // TODO hadzica: I acctualy figured out that we can solve that analytically istead of numercally like it is done here.
                    for (int j = 0; j < numberOfStepsToNextSwing; j++) {
                      desiredYawAtTimeStamp += dt * desiredYawRate;
                      // DEBUG
                      std::cout << "desiredYawAtTimeStamp = " << desiredYawAtTimeStamp << std::endl;
                      Matrix rotationMatrix(3, 3);

                      rotationMatrix << std::cos(desiredYawAtTimeStamp), 0., std::sin(desiredYawAtTimeStamp),
                        0., 1., 0.,
                        -std::sin(desiredYawAtTimeStamp), 0., std::cos(desiredYawAtTimeStamp);

                      auto velocityAtTimeStamp = rotationMatrix * desiredBaseVelocity;
                      auto differenceToPositionOfLastTimeTimeStamp = velocityAtTimeStamp * dt;
                      desiredBasePositionAtNextSwingState = desiredBasePositionAtNextSwingState + differenceToPositionOfLastTimeTimeStamp;

                      if (calculateDesiredBasePositionAtNextPhase && j < numberOfTimeStampsToNextPhase)
                        desiredBasePositionAtNextPhase = desiredBasePositionAtNextPhase + differenceToPositionOfLastTimeTimeStamp;

                      if (j == numberOfStepsToNextSwing - 1) {
                        orientationNextSwingState = rotationMatrix;
                      }
                    }

                    calculateDesiredBasePositionAtNextPhase = false;

                    auto targetFootholdPositionRelativeToInertialFrame = desiredBasePositionAtNextSwingState
                      + (orientationNextSwingState
                        * V3D(initialHorizontalDistanceVectorFromBaseToFootPositions[i]));
                    targetFootholdPositionRelativeToInertialFrame.y = 0;
                    targetFootholdPositions.push_back(targetFootholdPositionRelativeToInertialFrame);

                    // DEBUG
                    std::cout << "targetFootHoldPositionX = " << targetFootholdPositionRelativeToInertialFrame[0] << std::endl;
                    std::cout << "targetFootHoldPositionY = " << targetFootholdPositionRelativeToInertialFrame[1] << std::endl;
                    std::cout << "targetFootHoldPositionZ = " << targetFootholdPositionRelativeToInertialFrame[2] << std::endl;
                  }
                  else {
                    auto targetFootholdPosition = previousTargetFootholdPositions[i];
                    // Safety reasons.
                    targetFootholdPosition.y = 0.;
                    targetFootholdPositions.push_back(targetFootholdPosition);
                  }
                }
                else
                {
                  targetFootholdPositions.push_back(previousTargetFootholdPositions[i]);
                }
                    // Set very first reference.
                    desiredFootPositions.push_back(previousTargetFootholdPositions[i]);
                    desiredFootVelocities.push_back(P3D());
                    desiredFootAccelerations.push_back(P3D());
                    // Prepare for next phase.
                    desiredBasePositionNow = desiredBasePositionAtNextPhase;
                }
            }

            void updateSagittalAndCoronalTrajectoryCoefficients() {
                Coef_x.clear();
                Coef_z.clear();

                for(int i = 0; i < numberOfLegs; i++) {
                    if(!isInContact[i]) {
                        // solve system of equations to get 5th order polynomial coeff for x- and z-trajectory from start to end point
                        Matrix3x3 A_coef;
                        A_coef << 1., 1., 1.,
                                  5., 4., 3.,
                                  20., 12., 6.;

                        Vector3d b_coef_x, b_coef_z;
                        b_coef_x << (targetFootholdPositions[i].x - previousTargetFootholdPositions[i].x), 0., 0.;
                        b_coef_z << (targetFootholdPositions[i].z - previousTargetFootholdPositions[i].z), 0., 0.;

                        // std::cout << "b_coef_z: " << b_coef_z << std::endl;

                        Vector3d coef_x = A_coef.colPivHouseholderQr().solve(b_coef_x);
                        Vector3d coef_z = A_coef.colPivHouseholderQr().solve(b_coef_z);

                        // avoid numerical issues with very small numbers
                        if (coef_x.squaredNorm() < 1e-6)
                        {
                            coef_x = Vector3d::Zero();
                        }
                        if (coef_z.squaredNorm() < 1e-6)
                        {
                            coef_z = Vector3d::Zero();
                        }

                        // std::cout << "coef_x: " << coef_x << std::endl;
                        // std::cout << "coef_z: " << coef_z << std::endl;

                        Vector4d coefs_x;
                        coefs_x << coef_x, previousTargetFootholdPositions[i].x;
                        Coef_x.push_back(coefs_x);

                        Vector4d coefs_z;
                        coefs_z << coef_z, previousTargetFootholdPositions[i].z;
                        Coef_z.push_back(coefs_z);
                    }
                    else {
                        Coef_x.push_back(Vector4d::Zero());
                        Coef_z.push_back(Vector4d::Zero());
                    }
                }
            }

            void updateFootModes() {
                for(int i = 0; i < numberOfLegs; i++) {
                    isInContact[i] = gaitPattern[i][currentPhaseIterator];
                    // DEBUG
                    // std::cout << isInContact[i] << std::endl;
                }
            }

            void updateCurrentStride () {
                currentStride = (double(periodIterator) * dt) / desiredGatePeroid + double(strideIterator) * phaseDuration;   
                periodIterator++;
            }

            void interpolateFootTrajectories() {
                double swingStride = (currentStride - double(currentPhaseIterator) * phaseDuration) / phaseDuration;
                // DEBUG
                std::cout << "swingStride = " << swingStride << std::endl;
                desiredFootPositions.clear();
                desiredFootVelocities.clear();
                desiredFootAccelerations.clear();

             for (int i = 0; i < numberOfLegs; i++) {
              // DEBUG
              // std::cout << "Interpolation: Leg " << i << std::endl;
              if (footEnabled[i])
              {
              if (!isInContact[i]) {
                // TODO1: Use the now folrmula (note on paper) to calculate these 3 quantities. Note
                // that swing stride is "t".
                // auto targetFootPosition = LinearCoefficient_A[i] * swingStride + LinearCoefficient_b[i];
                P3D targetFootPosition;
                P3D targetFootVelocity;
                P3D targetFootAcceleration;

                // compute current foot control references in x
                Vector4d coefs_x = Coef_x[i];
                // std::cout << "coefs_x: " << coefs_x << std::endl;
                targetFootPosition.x = coefs_x(0) * pow(swingStride, 5) +
                  coefs_x(1) * pow(swingStride, 4) +
                  coefs_x(2) * pow(swingStride, 3) +
                  coefs_x(3);

                targetFootVelocity.x = 5. * coefs_x(0) * pow(swingStride, 4) +
                  4. * coefs_x(1) * pow(swingStride, 3) +
                  3. * coefs_x(2) * pow(swingStride, 2);

                targetFootAcceleration.x = 20. * coefs_x(0) * pow(swingStride, 3) +
                  12. * coefs_x(1) * pow(swingStride, 2) +
                  6. * coefs_x(2) * swingStride;

                // compute current foot control references in z
                Vector4d coefs_z = Coef_z[i];
                std::cout << "coefs_z: " << coefs_z << std::endl;
                targetFootPosition.z = coefs_z(0) * pow(swingStride, 5) +
                  coefs_z(1) * pow(swingStride, 4) +
                  coefs_z(2) * pow(swingStride, 3) +
                  coefs_z(3);

                targetFootVelocity.z = 5. * coefs_z(0) * pow(swingStride, 4) +
                  4. * coefs_z(1) * pow(swingStride, 3) +
                  3. * coefs_z(2) * pow(swingStride, 2);

                targetFootAcceleration.z = 20. * coefs_z(0) * pow(swingStride, 3) +
                  12. * coefs_z(1) * pow(swingStride, 2) +
                  6. * coefs_z(2) * swingStride;

                // std::cout << "targetFootPosition = " << LinearCoefficient_A[i].z << " * " << swingStride << " + " << LinearCoefficient_b[i].z << std::endl;

                // auto targetFootVelocity = LinearCoefficient_A[i]; 
                // auto targetFootAcceleration = P3D();
                // TODO hadzica: Comment
                // compute current foot control references in y
                targetFootPosition.y = swingStride * swingStride * swingStride * (aHeight * swingStride * swingStride * swingStride
                  + bHeight * swingStride * swingStride + cHeight * swingStride + dHeight);
                targetFootVelocity.y = swingStride * swingStride * (6. * aHeight * swingStride * swingStride * swingStride
                  + 5. * bHeight * swingStride * swingStride + 4. * cHeight * swingStride + 3. * dHeight);
                targetFootAcceleration.y = (swingStride * (30. * aHeight * swingStride * swingStride * swingStride
                  + 20. * bHeight * swingStride * swingStride + 12. * cHeight * swingStride + 6. * dHeight));

                if (abs(targetFootAcceleration.y - previousdesiredFootYAccerlations[i]) > 0.4) {
                  // TODO steffeol: We should move this saftey layer down to PD + FF controller in
                  // RBControl class. In addition, we should also include a saftey layer for x and z direction.
                  // Once this is implemented we can delete this section.
                  std::cout << "I'm helping you stabilizing your shit you can't handle" << std::endl;
                  if ((targetFootAcceleration.y - previousdesiredFootYAccerlations[i]) > 0) {
                    targetFootAcceleration.y = previousdesiredFootYAccerlations[i] + 0.4;
                  }
                  else {
                    targetFootAcceleration.y = previousdesiredFootYAccerlations[i] - 0.4;
                  }

                }

                desiredFootPositions.push_back(targetFootPosition);
                desiredFootVelocities.push_back(targetFootVelocity);
                desiredFootAccelerations.push_back(targetFootAcceleration);
                previousdesiredFootYAccerlations[i] = targetFootAcceleration.y;
              }
              else {
                desiredFootPositions.push_back(targetFootholdPositions[i]);
                desiredFootVelocities.push_back(P3D());
                desiredFootAccelerations.push_back(P3D());
              }
              }
              else
              {
                desiredFootPositions.push_back(previousTargetFootholdPositions[i]);
                desiredFootVelocities.push_back(P3D());
                desiredFootAccelerations.push_back(P3D());
              }
                }
            }

            void computeDesiredBasePosition(const double & stanceWeight, const double & swingWeight) {
                desiredBasePosition = P3D();
                double sumOfWeights = 0.;
				double weight = stanceWeight;
                for(int i = 0; i < numberOfLegs; i++) {
					if (footEnabled)
					{
						if (!isInContact[i]) weight = swingWeight;
						desiredBasePosition = desiredBasePosition + (desiredFootPositions[i] * weight);
						sumOfWeights += weight;
					}
					else
					{
						desiredBasePosition = desiredBasePosition + (initialHorizontalDistanceVectorFromBaseToFootPositions[i] * weight);
					}
                    
                }
                desiredBasePosition /= sumOfWeights;
                desiredBasePosition.y = referenceBaseHeight;
				// desiredBasePosition.x += 0.03;
				// desiredBasePosition.z -= 0.03;
            }   

            void forceContact() {
                desiredFootPositions.clear();
                desiredFootVelocities.clear();
                for(int i = 0; i < numberOfLegs; i++) {
					if (footEnabled[i])
					{
						auto desiredFootPosition = targetFootholdPositions[i];
						desiredFootPosition.y = 0.;
						desiredFootPositions.push_back(desiredFootPosition);
						desiredFootVelocities.push_back(P3D());
						desiredFootAccelerations[i].x = 0.;
						desiredFootAccelerations[i].z = 0.;
						// todo steffeol: Another reason why throttling at
						// PD + FF level is better that these nasty if statements are not neccessary
						// anymore. We can simply do the -= whitout checking since the PD + FF will
						// handle that for us. As addition, we can also integrate the height velocity and
						// the height forward in time to generate smooth trajectories even at contact search.
						// Maybe this would improve it a bit. 
						if (desiredFootAccelerations[i].y > -0.3)
						{
							desiredFootAccelerations[i].y -= 0.15;

						}
						if (desiredFootAccelerations[i].y < -0.3)
						{
							desiredFootAccelerations[i].y = -0.3;
						}
						computeDesiredBasePosition(stanceWeight, stanceWeight);
					}
					else
					{
						desiredFootPositions.push_back(previousTargetFootholdPositions[i]);
						desiredFootVelocities.push_back(P3D());
						desiredFootAccelerations[i].x = 0.;
						desiredFootAccelerations[i].y = 0.;
						desiredFootAccelerations[i].z = 0.;
					}
                }
                // During waiting for contact we don't want that base continues to travel with 
                // desired velocity. Otherwise the relative positions after swing phase don't match anymore.
                desiredBaseVelocity = V3D(); 
                desiredYawRate = 0.;
            }

            bool contactChecked() {
                updateFootPositions();
                int i = 0;
                bool decision = true;
                for(const auto footPosition : footPositions) {
					if (footEnabled[i])
					{
						double height = footPosition.y;
						// DEBUG 
						// std::cout << "Height Of foot " << i << " = " << height << std::endl;
						// std::cout << "Height Of foot " << i << "is smaller " << (height <= contactTolerance) << std::endl;
						decision *= (height <= contactTolerance);
						i++;
					}
                }
                // DEBUG
                std::cout << "All feet are in contact is " << decision << std::endl;
                return decision;
            }
            
            // Robot and ridid bodies
            Robot* robot;
            RobotRB* base;
            std::vector<RobotRB*> footRBs; 

            // Parameters and constants
            static const int numberOfPhases = 6;  
            static const int numberOfLegs = 6;
            const int footRBsStartingIndex = 13;
            double stanceWeight = .8;
            double swingWeight = .8;
            double contactTolerance = 0.999 * 1e-3;

            // Target and desired quantities
            std::vector<P3D> targetFootholdPositions;
            std::vector<P3D> previousTargetFootholdPositions;
            std::vector<double> previousdesiredFootYAccerlations;
            std::vector<P3D> desiredFootPositions;
            std::vector<P3D> desiredFootVelocities;
            std::vector<P3D> desiredFootAccelerations;
            std::vector<P3D> initialHorizontalDistanceVectorFromBaseToFootPositions;
            std::vector<P3D> orthogonalUnitVectorToInitialHorizontalDistanceBaseToFootPositions;
            P3D desiredBasePositionNow;
            P3D desiredBasePosition;
            V3D desiredBaseVelocity;
            double referenceBaseHeight;
            double desiredYawRate;
            double desiredYaw;

            // State variables
            std::vector<P3D> footPositions;
            P3D basePosition;
            V3D baseVelocity;
            
            // Meta level state variables
            bool isInContact[numberOfLegs];     
            bool gaitPattern[numberOfLegs][numberOfPhases];
			      bool footEnabled[numberOfLegs];
            
            //number of swings in period
            int numberOfSwingsInPeriod[numberOfLegs];
            
            // Timer relevant variables
            double currentStride;
            int currentPhaseIterator;
            int strideIterator;
            int periodIterator; 
            const double phaseDuration = 1. / double(numberOfPhases);
            // TODO3: Move that to GUI, attention: Let that varible only be updated, when a period is done. hadzica: good point
            const double desiredGatePeroid = 6.;// 4.;//6. * .5;
            const double dt;
            // TODO3: Attention, when desiredGatePeroid is moved to gui. This variable has
            // to be updated if desiredGatePeroid is updated. 
            const double deltaT = phaseDuration * desiredGatePeroid;
            const int numberOfTimeStampsToNextPhase;
			      const double g = 9.81;

            // Trajectory coefficients
            std::vector<Eigen::Vector4d> Coef_x;
            std::vector<Eigen::Vector4d> Coef_z;

            // See Matlab file how this was calculated.
            const double aHeight = - 96. / 125.;
            const double bHeight = 288. / 125.;
            const double cHeight = - 288. / 125.;
            const double dHeight = 96. / 125.;
            
            // flags
            bool isInit;
            bool strideIteratorUpdated;

            // period counter
            int periodcounter = 0;

            // debug
            double desiredYawAtTimeStamp;
    };
}