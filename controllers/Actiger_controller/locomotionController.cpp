#include "locomotionController.hpp"
#include <iostream>

using namespace quadruped;

LocomotionCtrl::LocomotionCtrl(void)
{
	para = new Parameters();
	robotModel = new RobotModel(para);
	motionPlan = new MotionPlan(robotModel, para);
	legControl = new LegControl(robotModel, motionPlan, para);
	mpc = new Mpc(robotModel,motionPlan,para);

	std::cout << "LocomotionCtrl Init" << std::endl;
}


LocomotionCtrl::~LocomotionCtrl(void)
{
	delete para;
	delete robotModel;
	delete motionPlan;
	delete legControl;
	delete mpc;
}


void LocomotionCtrl::RobotControl(void)
{
	robotModel->UpdataModel(mpc_pre_force, mpc_pre_pre_force);
	
	mpc_pre_pre_force=mpc_pre_force;
	mpc->GetReactForce();

	mpc_pre_force.block(0,0,3,1) = motionPlan->legPlan[0]->fr_des;
	mpc_pre_force.block(3,0,3,1) = motionPlan->legPlan[1]->fr_des;
	mpc_pre_force.block(6,0,3,1) = motionPlan->legPlan[2]->fr_des;
	mpc_pre_force.block(9,0,3,1) = motionPlan->legPlan[3]->fr_des;

	legControl->LegJointControl();
}