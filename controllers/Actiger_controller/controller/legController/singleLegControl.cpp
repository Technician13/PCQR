#include "singleLegControl.hpp"
#include <iostream>

using namespace quadruped;

SingleLeg::SingleLeg(LegStateEstimate *p_legStste, Parameters *p_para)
{
	legStste = p_legStste;
	para = p_para;

	std::cout << "SingleLeg Init" << std::endl;
}


SingleLeg::~SingleLeg() {}


legTorque_t SingleLeg::SingleStanceControl(Mat3<double> p_rotation, Vec3<double> p_grav, Vec3<double> coriolis, Vec3<double> p_reactForce)
{
	legTorque_t return_torque = {0};
	Vec3<double> torque = Vec3<double>::Zero();

	torque = -legStste->jacobian.transpose() * p_rotation * p_reactForce + p_grav + coriolis;

	return_torque.shoulder_torque = torque(0);
	return_torque.hip_torque = torque(1);
	return_torque.knee_torque = torque(2);

	return return_torque;
}


legTorque_t SingleLeg::SingleSwingControl(Mat3<double> p_rotation, Vec3<double> p_grav, Vec3<double> coriolis, Vec3<double> p_desPos, Vec3<double> p_desVel)
{
	legTorque_t return_torque = {0};
	Vec3<double> toreuqe = Vec3<double>::Zero();
	Vec3<double> force = Vec3<double>::Zero();

	Vec3<double> desPos = p_rotation * (p_desPos)-legStste->hip_pos_body;
	Vec3<double> desVel = p_rotation * p_desVel;

	force(0) = para->leg_kpJoint(0) * (desPos(0) - legStste->endfoot_hip(0)) + para->leg_kdJoint(0) * (desPos(0) - legStste->endfoot_vel_hip(0));
	force(1) = para->leg_kpJoint(1) * (desPos(1) - legStste->endfoot_hip(1)) + para->leg_kdJoint(1) * (desPos(1) - legStste->endfoot_vel_hip(1));
	force(2) = para->leg_kpJoint(2) * (desPos(2) - legStste->endfoot_hip(2)) + para->leg_kdJoint(2) * (desPos(2) - legStste->endfoot_vel_hip(2));

	toreuqe = legStste->jacobian.transpose() * force + p_grav + coriolis;

	return_torque.shoulder_torque = toreuqe(0);
	return_torque.hip_torque = toreuqe(1);
	return_torque.knee_torque = toreuqe(2);

	return return_torque;
}