#include "robotModel.hpp"
#include <iostream>

using namespace quadruped;

RobotModel::RobotModel(Parameters *p_para)
{
	model = new FloatingBaseModel();
	model->PrintLinkList();

	lf_state = new LegStateEstimate(LF);
	lr_state = new LegStateEstimate(LR);
	rf_state = new LegStateEstimate(RF);
	rr_state = new LegStateEstimate(RR);
	bodyState = new BodyStateEstimate();

	legState.emplace_back(lf_state);
	legState.emplace_back(lr_state);
	legState.emplace_back(rf_state);
	legState.emplace_back(rr_state);

	lf_hip_pos_body = p_para->lf_hip_pos_body;
	lr_hip_pos_body = p_para->lr_hip_pos_body;
	rf_hip_pos_body = p_para->rf_hip_pos_body;
	rr_hip_pos_body = p_para->rr_hip_pos_body;

	pos_vel_estimator = new LinearKalmanFilter();
	pos_vel_estimator->Init();

	std::cout << "RobotModel Init" << std::endl;
}


RobotModel::~RobotModel(void)
{
	delete bodyState;
	delete lf_state;
	delete lr_state;
	delete rf_state;
	delete rr_state;

	typename std::vector<LegStateEstimate *>::iterator iter = legState.begin();
	while (iter < legState.end())
	{
		delete (*iter);
		++iter;
	}
	legState.clear();
}


void RobotModel::Resize(void)
{
	legState.clear();
	linkEndPos_world.clear();
	linkEndVel_world.clear();
	linkEndPos.clear();
	linkEndVel.clear();
	J.clear();
	Jdqd.clear();

	q_now.setZero();
	q_dot_now.setZero();
	A.setZero();
	Ainv.setZero();
	grav.setZero();
	coriolis.setZero();
}


Vec3<double> RobotModel::CalEndfootPos2Body(void) {}


Vec3<double> RobotModel::CalEndfootVel2Body(void) {}


void RobotModel::UpdataModel(Vec12<double> p_mpc_force, Vec12<double> p_mpc_force_1)
{
	pthread_mutex_lock(&model_mutex);
	Resize();
	robotLegMsg_t robotLegMsg = GetRobotJointMsg();
	legState[0]->GetLegMsg(robotLegMsg.lf);
	legState[1]->GetLegMsg(robotLegMsg.lr);
	legState[2]->GetLegMsg(robotLegMsg.rf);
	legState[3]->GetLegMsg(robotLegMsg.rr);
	bodyState->GetRobotMsg(lf_state, lr_state, rf_state, rr_state, p_mpc_force, p_mpc_force_1, Vec3<double>::Zero(3));
	
	Vec3<double> pos, vel;
	GetRobotBodyVel(pos, vel);

	bodyState->actul_body_pos = pos;
	bodyState->actul_body_vel = vel;

	bodyEuler = bodyState->actul_body_angle;
	bodyEulerAngularVel = bodyState->actul_body_angularVel;
	bodyPosition = bodyState->actul_body_pos;
	bodyVelocity = bodyState->actul_body_vel;

	dynacore::convert(bodyEuler[2], bodyEuler[1], bodyEuler[0], qua);
	Vec3<double> q3 = Vec3<double>::Zero();
	q3 << qua.x(), qua.y(), qua.z();

	q_now.block(0, 0, 3, 1) = bodyPosition;
	q_now.block(3, 0, 3, 1) = q3;
	q_now.block(6, 0, 3, 1) = legState[LF]->joint_angle;
	q_now.block(9, 0, 3, 1) = legState[LR]->joint_angle;
	q_now.block(12, 0, 3, 1) = legState[RF]->joint_angle;
	q_now.block(15, 0, 3, 1) = legState[RR]->joint_angle;
	q_now(18, 0) = qua.w();

	q_dot_now.block(0, 0, 3, 1) = bodyVelocity;
	q_dot_now.block(3, 0, 3, 1) = bodyEulerAngularVel;
	q_dot_now.block(6, 0, 3, 1) = legState[LF]->joint_anglularVel;
	q_dot_now.block(9, 0, 3, 1) = legState[LR]->joint_anglularVel;
	q_dot_now.block(12, 0, 3, 1) = legState[RF]->joint_anglularVel;
	q_dot_now.block(15, 0, 3, 1) = legState[RR]->joint_anglularVel;

	state_now.block(0, 0, 3, 1) = bodyEuler;
	state_now.block(3, 0, 3, 1) = bodyPosition;
	state_now.block(6, 0, 3, 1) = legState[LF]->joint_angle;
	state_now.block(9, 0, 3, 1) = legState[LR]->joint_angle;
	state_now.block(12, 0, 3, 1) = legState[RF]->joint_angle;
	state_now.block(15, 0, 3, 1) = legState[RR]->joint_angle;

	model->UpdateSystem(q_now, q_dot_now);
	model->getWorld2BodyMatrix(world2BodyRotation);
	body2WorldRotation = dynacore::RotZ_Matrix(bodyEuler[2]) * dynacore::RotY_Matrix(bodyEuler[1]) * dynacore::RotX_Matrix(bodyEuler[0]);

	Vec3<double> linkPos = Vec3<double>::Zero();
	for (int i = 1; i <= 4; i++)
	{
		model->getPos(i, linkPos);
		linkEndPos_world.emplace_back(linkPos);
		legState[i - 1]->endfoot_world = linkPos;
		linkPos.setZero();
	}

	Vec3<double> linkVel = Vec3<double>::Zero();
	for (int i = 1; i <= 4; i++)
	{
		model->getLinearVel(i, linkVel); /**/
		linkEndVel_world.emplace_back(linkVel);
		legState[i - 1]->endfoot_vel_world = linkVel;
		linkVel.setZero();
	}

	DMat<double> jacobian = DMat<double>::Zero(6, robot_dim::num_qdot);
	for (int i = 1; i <= 4; i++)
	{
		model->getFullJacobian(i, jacobian);
		J.emplace_back(jacobian.block(3, 0, 3, robot_dim::num_qdot));
		jacobian.setZero();
	}

	DMat<double> A_;
	model->getMassInertia(A_);
	A = A_;

	cenInertia = A.block(0, 0, 6, 6);

	DVec<double> coriolis_;
	model->getCoriolis(coriolis_);
	coriolis = coriolis_;

	DVec<double> grav_;
	model->getGravity(grav_);
	grav = grav_;
	pthread_mutex_unlock(&model_mutex);
}