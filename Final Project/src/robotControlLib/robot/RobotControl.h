#pragma once

#include <robot/Robot.h>
#include <robot/GeneralizedCoordinatesRobotRepresentation.h>
#include <utils/utils.h>
#include <optLib/SQPMinimizer.h>
#include <robot/SQPobjective.h>
#include <robot/RBControl.h>

double inf = std::numeric_limits<double>::infinity();

enum RobotControlMode {
	GUI_SLIDER = 0,
	GAIT = 1
};

class RobotControl {
	// allow app to set protected & private members
	friend class App;
public:
	// constructor
	RobotControl(Robot* a) :
		// initializer list
		robot(a), 
		genCoords(a), 
		base(a->getRigidBody(0)), 
		footRF(a->getRigidBody(13)),
		footLF(a->getRigidBody(14)), 
		footLM(a->getRigidBody(15)),
		footRM(a->getRigidBody(16)),
		footRH(a->getRigidBody(17)),
		footLH(a->getRigidBody(18))
	{
		// some dimensions for matrix sizing
		numberOfGeneralizedCoordinates = genCoords.getDimensionSize();
		numberOfJoints = numberOfGeneralizedCoordinates - 6;
		numberOfFeet = 6;
		numberOfOptimizedVariables = numberOfGeneralizedCoordinates + 3 * numberOfFeet + numberOfJoints;

		// initialization of state vector (ddq at 0 and F_c and tau at 1)
		p.resize(numberOfOptimizedVariables);
		p << dVector::Zero(numberOfGeneralizedCoordinates), dVector::Ones(3 * numberOfFeet + numberOfJoints);

		// set (static) selection matrix for this robot
		S.resize(numberOfJoints, numberOfGeneralizedCoordinates);
		S << Matrix::Zero(numberOfJoints, 6), Matrix::Identity(numberOfJoints, numberOfJoints);
		// DEBUG
	    // std::cout << "The matrix S is of size " << S.rows() << "x" << S.cols() << std::endl;
	    // std::cout << S << std::endl;
	}

	// destructor
	~RobotControl() {}

	// method to get control torque for either (i) given slider offsets or (ii) given desired positions of base and EE in world frame
	dVector getControlTorque() {
		// goal: operational space control for IK by converting EOM and motion tasks to QPs as follows
		// - unknown: p = [ddq, F_c, tau]^T (|q + EE in contact with ground + q_j|x1)
		// - objective: 
		// 		- motion tasks of base and feet: A = [J, 0, 0], b = w^* - dJ * dq
		// - constraints:
		// 		- EOM: A = [M J_c^T -S^T], b = - C - g

		// sync genCoords with robot
		genCoords.syncGeneralizedCoordinatesWithRobotState();

		// get desired acceleration for base and feet
		updateControlReferences();

		// update constraints in solver with current values of genCoords
		updateConstraints();

		// update objective with current motion task
		updateObjective();

		// minimize objective
		minimizer.minimize(&objFunc, p);

		// recover torque commands from state
		dVector ddq = p.head(numberOfGeneralizedCoordinates);
		dVector F_c = p.segment(numberOfGeneralizedCoordinates, 3 * numberOfFeet);
		dVector tau = p.tail(numberOfJoints);
		// DEBUG
	    std::cout << "The vector ddq is of size " << ddq.size() << std::endl;
	    std::cout << ddq << std::endl;
	    std::cout << "The vector F_c is of size " << F_c.size() << std::endl;
	    std::cout << F_c << std::endl;
		std::cout << "The vector tau is of size " << tau.size() << std::endl;
	    std::cout << tau << std::endl;		

		return tau;
	}

	void updateControlReferences() {

		// feet
		footRFController.onGround = feet_on_ground(0);
		footLFController.onGround = feet_on_ground(1);
		footLMController.onGround = feet_on_ground(2);
		footRMController.onGround = feet_on_ground(3);
		footRHController.onGround = feet_on_ground(4);
		footLHController.onGround = feet_on_ground(5);

		if (controlMode == RobotControlMode::GUI_SLIDER)
		{
			// slider offsets determine control
			baseController.computeDesLinAcc(slider_ref_offsets.col(0));
			baseController.computeDesRotAcc(slider_ref_offsets.col(1));
			footRFController.computeDesLinAcc(slider_ref_offsets.col(2));
			footLFController.computeDesLinAcc(slider_ref_offsets.col(3));
			footLMController.computeDesLinAcc(slider_ref_offsets.col(4));
			footRMController.computeDesLinAcc(slider_ref_offsets.col(5));
			footRHController.computeDesLinAcc(slider_ref_offsets.col(6));
			footLHController.computeDesLinAcc(slider_ref_offsets.col(7));
		}
		else
		{
			// desired position and velocity by gait controller determine control
			baseController.computeDesLinAccGait(gait_des_pos.col(0), gait_des_vel.col(0), gait_des_acc.col(0));
			baseController.computeDesRotAccGait(gait_des_pos.col(1), gait_des_vel.col(1), gait_des_acc.col(1));
			footRFController.computeDesLinAccGait(gait_des_pos.col(2), gait_des_vel.col(2), gait_des_acc.col(2));
			footLFController.computeDesLinAccGait(gait_des_pos.col(3), gait_des_vel.col(3), gait_des_acc.col(3));
			footLMController.computeDesLinAccGait(gait_des_pos.col(4), gait_des_vel.col(4), gait_des_acc.col(4));
			footRMController.computeDesLinAccGait(gait_des_pos.col(5), gait_des_vel.col(5), gait_des_acc.col(5));
			footRHController.computeDesLinAccGait(gait_des_pos.col(6), gait_des_vel.col(6), gait_des_acc.col(6));
			footLHController.computeDesLinAccGait(gait_des_pos.col(7), gait_des_vel.col(7), gait_des_acc.col(7));
		}
	}

	void updateConstraints() {

		// update dynamics
		updateDynamics();

		// update constraint jacobian
		updateJacobians();

		// update constraint matrix
		updateConstraintMatrix();

		// update constraint target vector
		updateContstraintTarget();
	}

	void updateDynamics() {

		// update mass matrix
		genCoords.computeMassMatrix(M);
		// DEBUG
		// std::cout << "The matrix M is of size " << M.rows() << "x" << M.cols() << std::endl;

		// update coriolis term
		genCoords.computeCoriolisAndCentrifugalForcesTerm(Cor);
		// DEBUG
		// std::cout << "The term C is of size " << C.size() <<  std::endl;
		// std::cout << "look at C: " << C << std::endl;		

		// update gravity term 
		genCoords.computeGravityTerm(g);
		// DEBUG
		// std::cout << "The term g is of size " << g.size() <<  std::endl;
		// std::cout << "g:\n" << g << std::endl;		
	}

	void updateJacobians() {

		// get contact jacobian for EOM: zero if foot is in air
		resize(J_c, 3 * numberOfFeet, numberOfGeneralizedCoordinates);
		J_c.setZero();
		J_c << footRFController.getContactJac(), 
			   footLFController.getContactJac(),
			   footLMController.getContactJac(),
			   footRMController.getContactJac(),
			   footRHController.getContactJac(),
			   footLHController.getContactJac();
		// DEBUG
	    // std::cout << "The matrix J_c is of size " << J_c.rows() << "x" << J_c.cols() << std::endl;
	    // std::cout << J_c << std::endl;
	    // std::cout << "The matrix J_c_new is of size " << J_c_new.rows() << "x" << J_c_new.cols() << std::endl;
	    // std::cout << J_c_new << std::endl;

		// get end effector jacobian for objective
		resize(J_EE, 3 * numberOfFeet, numberOfGeneralizedCoordinates);
		J_EE.setZero();
		J_EE << footRFController.getLinJac(), 
			   	footLFController.getLinJac(),
			   	footLMController.getLinJac(),
			   	footRMController.getLinJac(),
			   	footRHController.getLinJac(),
			   	footLHController.getLinJac();

		// get derivative of end effector jacobian for objective
		resize(J_EE_dot, 3 * numberOfFeet, numberOfGeneralizedCoordinates);
		J_EE_dot.setZero();
		J_EE_dot << footRFController.getLinJacDot(), 
				   footLFController.getLinJacDot(),
			   	   footLMController.getLinJacDot(),
			       footRMController.getLinJacDot(),
			       footRHController.getLinJacDot(),
			       footLHController.getLinJacDot();
		// DEBUG
	    // std::cout << "The matrix J_EE_dot is of size " << J_EE_dot.rows() << "x" << J_EE_dot.cols() << std::endl;

		// get jacobians for base
		J_b = baseController.getLinJac();
		J_b_rot = baseController.getRotJac();
		// DEBUG
	    // std::cout << "The matrix J_b is of size " << J_b.rows() << "x" << J_b.cols() << std::endl;
	    // std::cout << J_b << std::endl;

		// get derivative of jacobian for base
		J_b_dot = baseController.getLinJacDot();
		J_b_rot_dot = baseController.getRotJacDot();
	}

	void updateConstraintMatrix() {

		// assemble equality constraint matrix for EOM
		Matrix A_eom(M.rows(), numberOfOptimizedVariables);
		A_eom << M, -J_c.transpose(), -S.transpose();
		// DEBUG
	    // std::cout << "The matrix A_eom is of size " << A_eom.rows() << "x" << A_eom.cols() << std::endl;
	    // std::cout << A_eom << std::endl;

		// assemble inequality constraint matrix for friction cones
		Matrix C = Matrix::Zero(5 * numberOfFeet, numberOfOptimizedVariables);
		double my = 0.8 / sqrt(2);
		for (int i = 0; i < (numberOfFeet); i++)
		{
			C(i * 5, numberOfGeneralizedCoordinates + i * 3) = 1.;				// 0 < my * F_cy + F_cx
			C(i * 5, numberOfGeneralizedCoordinates + i * 3 + 1) = my;
			C(i * 5 + 1, numberOfGeneralizedCoordinates + i * 3) = -1.;			// 0 < my * F_cy - F_cx
			C(i * 5 + 1, numberOfGeneralizedCoordinates + i * 3 + 1) = my;
			C(i * 5 + 2, numberOfGeneralizedCoordinates + i * 3 + 2) = 1.;		// 0 < my * F_cy + F_cz
			C(i * 5 + 2, numberOfGeneralizedCoordinates + i * 3 + 1) = my;
			C(i * 5 + 3, numberOfGeneralizedCoordinates + i * 3 + 2) = -1.;		// 0 < my * F_cy - F_cz
			C(i * 5 + 3, numberOfGeneralizedCoordinates + i * 3 + 1) = my;
			C(i * 5 + 4, numberOfGeneralizedCoordinates + i * 3 + 1) = 1.;		// 0 < F_cy;
		}

		// update SQPMinimizer (member matrices are sparse)
		minimizer.A = A_eom.sparseView();
		minimizer.C = C.sparseView();
	}

	void updateContstraintTarget() {

		// assemble target vector for EOM
		dVector b_eom = - Cor - g;
		// DEBUG
	    // std::cout << "The matrix b_eom is of size " << b_eom.rows() << "x" << b_eom.cols() << std::endl;

		// update objective
		objFunc.b = b_eom;

		// update inequality constraint vectors
		objFunc.d = dVector::Zero(5 * numberOfFeet);
		objFunc.f = dVector::Ones(5 * numberOfFeet) * inf;

		// update state boundaries
		objFunc.l = dVector::Ones(numberOfOptimizedVariables) * -inf;					
		objFunc.l.tail(numberOfJoints) = dVector::Ones(numberOfJoints)  * -100.;	// tau >= - tau_max
		//objFunc.l(numberOfOptimizedVariables - numberOfJoints + 0) = -0.01;
		//objFunc.l(numberOfOptimizedVariables - numberOfJoints + 6) = -0.01;
		//objFunc.l(numberOfOptimizedVariables - numberOfJoints + 12) = -0.01;

		objFunc.u = dVector::Ones(numberOfOptimizedVariables) * inf;					
		objFunc.u.tail(numberOfJoints) = dVector::Ones(numberOfJoints)  * 100.;	// tau <= tau_max
		//objFunc.u(numberOfOptimizedVariables - numberOfJoints + 0) = 0.01;
		//objFunc.u(numberOfOptimizedVariables - numberOfJoints + 6) = 0.01;
		//objFunc.u(numberOfOptimizedVariables - numberOfJoints + 12) = 0.01;
	}

	void updateObjective() {

		// assemble objective matrix for motion task of base
		Matrix A_base(J_b.rows() + J_b_rot.rows(), numberOfOptimizedVariables);
		A_base << J_b, Matrix::Zero(3, numberOfOptimizedVariables - numberOfGeneralizedCoordinates), 
				  J_b_rot, Matrix::Zero(3, numberOfOptimizedVariables - numberOfGeneralizedCoordinates);

		// assemble objective matrix for motion tasks of feet
		Matrix A_feet(J_EE.rows(), numberOfOptimizedVariables);
		A_feet  << J_EE, Matrix::Zero(J_EE.rows(), numberOfOptimizedVariables - J_EE.cols());

		// assemble objective matrix
		Matrix A(A_base.rows() + A_feet.rows(), numberOfOptimizedVariables);
		A << A_base, A_feet;
		// DEBUG
	    // std::cout << "The matrix A is of size " << A.rows() << "x" << A.cols() << std::endl;
	    // std::cout << A << std::endl;

		dVector qDot;
		genCoords.getQDot(qDot);

		// assemble objective target vector for base (linear and rotational)
		dVector b_base = baseController.desLinAcc - J_b_dot * qDot;
		dVector b_base_rot = baseController.desRotAcc - J_b_rot_dot * qDot;

		// assemble objective target vector for feet (only linear)		
		dVector b_feet(3 * numberOfFeet);
		b_feet << footRFController.desLinAcc,
				  footLFController.desLinAcc,
				  footLMController.desLinAcc,
				  footRMController.desLinAcc,
				  footRHController.desLinAcc,
				  footLHController.desLinAcc;

		b_feet -= J_EE_dot * qDot;

		// assemble objective target vector
		dVector b(b_base.rows() + b_base_rot.rows() + b_feet.rows());
		b << b_base, b_base_rot, b_feet;
		// DEBUG
	    // std::cout << "The matrix b is of size " << b.rows() << "x" << b.cols() << std::endl;
	    // std::cout << b << std::endl;

		objFunc.setObjectiveMatrixAndTargetVector(A, b);
	}

private:
	// references to access robot
	Robot* robot;

	// object to access generalized coords of robot
	GeneralizedCoordinatesRobotRepresentation genCoords;

	// object to solve sequential quadratic programs for IK (good description in header file)
	SQPMinimizer minimizer;

	// constrained objective function to be solved by SQPMinimizer
	SQPobjective objFunc = SQPobjective(&minimizer);

	// some dimensions for matrix sizing
	int numberOfGeneralizedCoordinates;	// # generalized coords (base + joints)
	int numberOfJoints;	// # joints
	int numberOfFeet;	// # feet on ground (will be set by gait controller in future)
	int numberOfOptimizedVariables;

	// state vector
	dVector p;

	// objects required for computation of A, b, C, and d:
	Matrix M;	// mass matrix of robot (|q|x|q|)
	Matrix S;	// selection matrix selecting the actuated joints, i.e., everything except for base (|q|x|6+q|)
	dVector Cor;	// coriolis term (|q|x|1|)
	dVector g;  // gravity term (|q|x|1|)

	Matrix J_b;
	Matrix J_b_dot;
	Matrix J_b_rot;
	Matrix J_b_rot_dot;
	Matrix J_c;
	Matrix J_EE;
	Matrix J_EE_dot;

	// PD controller gains for base
	double kp_base = 10.;
	double kd_base = 5.;

	// PD controller gains for feet
	double kp_feet = 10.;
	double kd_feet = 5.;

	RobotControlMode controlMode = RobotControlMode::GUI_SLIDER;
	Matrix slider_ref_offsets;
	dVector feet_on_ground;
	Matrix gait_des_pos;
	Matrix gait_des_vel;
	Matrix gait_des_acc;

	// references to relevant RBs and their controllers
	// todo steffeol: Here is the location where you should add the limits.
	RobotRB* base;
	BaseControl baseController = BaseControl(base, &genCoords, kp_base, kd_base);

	RobotRB* footRF;	// rigth-front
	FootControl footRFController = FootControl(footRF, &genCoords, kp_feet, kd_feet);

	RobotRB* footLF;	// LF
	FootControl footLFController = FootControl(footLF, &genCoords, kp_feet, kd_feet);

	RobotRB* footLM;	// left-mid
	FootControl footLMController = FootControl(footLM, &genCoords, kp_feet, kd_feet);

	RobotRB* footRM;	// RM
	FootControl footRMController = FootControl(footRM, &genCoords, kp_feet, kd_feet);

	RobotRB* footRH;	// RH
	FootControl footRHController = FootControl(footRH, &genCoords, kp_feet, kd_feet);

	RobotRB* footLH;	// left-hind
	FootControl footLHController = FootControl(footLH, &genCoords, kp_feet, kd_feet);

};