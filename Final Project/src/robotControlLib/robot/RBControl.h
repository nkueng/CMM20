#pragma once

class RBControl {
// allow RobotControl to set and get protected members (desLinAcc)
	friend class RobotControl;
public:
	RBControl(RobotRB* a, GeneralizedCoordinatesRobotRepresentation* genCoords, double kp, double kd) : thisRB(a), genCoords(genCoords), kp(kp), kd(kd) {
		defPos_set = false;
		numberOfGeneralizedCoordinates = genCoords->getDimensionSize();
	}

	~RBControl() {}

	virtual P3D getWorldPos() = 0;

	virtual P3D getLocalPos() = 0;

	virtual V3D getLinVel() = 0;

	// used by slider reference
	void computeDesLinAcc(dVector ref_offset) {

		// DEBUG
		// std::cout << "The vector ref_offset is of size " << ref_offset.size() << std::endl;
	    // std::cout << ref_offset << std::endl;

		// simple PD controller on reference position from GUI
		P3D currPos = getWorldPos();
		// DEBUG
		// std::cout << "currPos: " << std::endl;
	 	// std::cout << currPos.x << ", " << currPos.y << ", " << currPos.z << std::endl;

		// set default position on first use of this function
		if (!defPos_set) {

			defPos = currPos;
			defPos_set = true;
		}
		// DEBUG
	 	// std::cout << "defPos: " << std::endl;
	 	// std::cout << defPos.x << ", " << defPos.y << ", " << defPos.z << std::endl;

		// get desired position from default position and offset from GUI
		P3D desPos = defPos + ref_offset;
		// DEBUG
	 	// std::cout << "desPos: " << std::endl;
		// std::cout << desPos.x << ", " << desPos.y << ", " << desPos.z << std::endl;

		// output desired acceleration from PD controller on error vector
		desLinAcc = getLinPDcontrol(desPos, V3D(0., 0., 0.), V3D(0., 0., 0.));
	}

	// used by gait reference
	void computeDesLinAccGait(dVector desPos, dVector desVel, dVector desAcc) {
		desLinAcc = getLinPDcontrol(getP3D(desPos), V3D(desVel), V3D(desAcc));
	}

	V3D getLinPDcontrol(P3D desPos, V3D desVel, V3D desAcc) {
		// TODO steffeol: Follow up of todo in motion generator. I was thinking of restricting
		// the acceleration here. Since this is the last step in the pipeline which feeds the values 
		// to the solver. Note, that we should add a new member in rigid body control class to
		// set each limits individually since for the base, front, mid and backlegs, the limits are different.
		// This method here is shared by all rigid bodies and therefore we cannot set the limit here hardcoded.
		// Thats why we need a new member. Don't forget that you should pass these limits in RobotControl class
		// at time where RBControlers are constructed.
		V3D err = V3D(getWorldPos(), desPos);
		// std::cout << "LinPDControl = " << kp * err + kd * (desVel - getLinVel()) + desAcc << std::endl;
		return kp * err + kd * (desVel - getLinVel()) + desAcc;
	}

	Matrix getLinJac() {

		Matrix linJac;
		genCoords->compute_dpdq(getLocalPos(), thisRB, linJac);
		return linJac;
	}

	Matrix getLinJacDot() {

		Matrix linJacDot;
		genCoords->compute_dpdq_dot(getLocalPos(), thisRB, linJacDot);
		return linJacDot;
	}

	Matrix getRotJac() {

		Matrix rotJac;
		genCoords->compute_angular_jacobian(thisRB, rotJac);
		return rotJac;
	}

	Matrix getRotJacDot() {

		Matrix rotJacDot;
		genCoords->compute_angular_jacobian_dot(thisRB, rotJacDot);
		return rotJacDot;
	}

	// Matrix getObjMat() {}

	// dVector getObjVec() {}

protected:
	RobotRB* thisRB;
	GeneralizedCoordinatesRobotRepresentation* genCoords;
	int numberOfGeneralizedCoordinates;
	double kp;
	double kd;

	// default/zero position of this RB
	P3D defPos;
	bool defPos_set;	
	V3D desLinAcc;
};

class BaseControl : public RBControl {
public:
	BaseControl(RobotRB* a, GeneralizedCoordinatesRobotRepresentation* genCoords, double kp, double kd) : RBControl(a, genCoords, kp, kd) {
		defOrient_set = false;
	}

	~BaseControl() {}

	V3D desRotAcc;
	Matrix3x3 defOrient;
	bool defOrient_set;

	P3D getWorldPos() {
		return thisRB->state.getWorldCoordinates(P3D(0., 0., 0.));
	}

	P3D getLocalPos() {
		return P3D(0., 0., 0.);
	}

	V3D getLinVel() {
		return thisRB->state.velocity;
	}

	Matrix3x3 getWorldOrientMat() {
		return (genCoords->getOrientationFor(thisRB)).normalized().toRotationMatrix();
	}

	Quaternion getWorldOrientQuat() {
		return (genCoords->getOrientationFor(thisRB)).normalized();
	}

	V3D getRotVel() {
		return genCoords->getAngularVelocityFor(thisRB);
	}

	V3D getRotPDcontrol(V3D err, V3D desRotVel, V3D desRotAccFF) {
		return .5 * kp * err + kd * (desRotVel - getRotVel()) + desRotAccFF;
	}

	void computeDesRotAcc(dVector ref_angles) {

		// simple PD controller on reference orientation from GUI
		Matrix3x3 currOrient = getWorldOrientMat();

		// set default orientation on first use of this function
		if (!defOrient_set) {

			defOrient = currOrient;
			defOrient_set = true;
		}

		// convert RPY-offset from GUI to rotation matrix
		Matrix3x3 ref_rot;
		ref_rot = Eigen::AngleAxisd(ref_angles[0], Vector3d::UnitX()) *
				  Eigen::AngleAxisd(ref_angles[1], Vector3d::UnitY()) *
				  Eigen::AngleAxisd(ref_angles[2], Vector3d::UnitZ());

		// std::cout << "ref_rot: \n" << ref_rot << std::endl;

		// get desired orientation from default orientation and offset from GUI
		Matrix3x3 desOrient = ref_rot * defOrient;

		// compute rotational error in angle axis representation
		Eigen::AngleAxisd deltaPhi(desOrient * currOrient.transpose());
		V3D err = deltaPhi.angle() * V3D(deltaPhi.axis());

		// output desired acceleration from PD controller on error vector
		desRotAcc = getRotPDcontrol(err, V3D(0., 0., 0.), V3D(0., 0., 0.));
		// std::cout << "desRotAcc: \n" << desRotAcc << std::endl;
	}

	void computeDesRotAccGait(dVector desOrientVec, dVector desRotVel, dVector desRotAccFF) {

		// simple PD controller on reference yaw velocity and acceleration from gait controller as well as zero orientation reference on roll and pitch
		Matrix3x3 currOrient = getWorldOrientMat();

		// get desired orientation from gait controller, vector contains Euler angles
		Matrix3x3 desOrient;
		desOrient = Eigen::AngleAxisd(desOrientVec[0], Vector3d::UnitX()) *
				    Eigen::AngleAxisd(desOrientVec[1], Vector3d::UnitY()) *
				    Eigen::AngleAxisd(desOrientVec[2], Vector3d::UnitZ());

		// std::cout << "desOrient: \n" << desOrient << std::endl;

		// compute rotational error in angle axis representation
		Eigen::AngleAxisd deltaPhi(desOrient * currOrient.transpose());
		V3D err = deltaPhi.angle() * V3D(deltaPhi.axis());

		// output desired acceleration from PD controller
		desRotAcc = getRotPDcontrol(err, V3D(desRotVel), V3D(desRotAccFF));
		// std::cout << "desRotVel: \n" << desRotVel << std::endl;
		// std::cout << "desRotAccFF: \n" << desRotAccFF << std::endl;
		// std::cout << "desRotAcc: \n" << desRotAcc << std::endl;

		// desRotAcc = V3D(0., 0., 0.);
	}

};

class FootControl : public RBControl {
public:
	FootControl(RobotRB* a, GeneralizedCoordinatesRobotRepresentation* genCoords, double kp, double kd) : RBControl(a, genCoords, kp, kd) {}

	~FootControl() {}

	bool onGround = true;

	P3D getWorldPos() {
		return thisRB->state.getWorldCoordinates(localEndEffectorPos);
	}

	P3D getLocalPos() {
		return localEndEffectorPos;
	}

	V3D getLinVel() {
		return thisRB->state.getVelocityForPoint_local(getLocalPos());
	}

	// disable contact constraint with zero-matrix if foot is in air
	Matrix getContactJac() {
		Matrix J_c;
		if (onGround)
		{
			J_c = getLinJac();
		}
		else
		{
			J_c = Matrix::Zero(3, numberOfGeneralizedCoordinates);
		}
		return J_c;
	}

private:
	P3D localEndEffectorPos = thisRB->rbProps.endEffectorPoints[0];
};