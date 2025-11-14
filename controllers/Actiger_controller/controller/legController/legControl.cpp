#include "legControl.hpp"
#include <iostream>

using namespace quadruped;

LegControl::LegControl(RobotModel *p_robotModel, MotionPlan *p_motionPlan, Parameters *p_para)
{
	lf_leg = new SingleLeg(p_robotModel->lf_state, p_para);
	lr_leg = new SingleLeg(p_robotModel->lr_state, p_para);
	rf_leg = new SingleLeg(p_robotModel->rf_state, p_para);
	rr_leg = new SingleLeg(p_robotModel->rr_state, p_para);

	robotModel = p_robotModel;
	motionPlan = p_motionPlan;

	std::cout << "legControl Init" << std::endl;
}


LegControl::~LegControl()
{
	delete lf_leg;
	delete lr_leg;
	delete rf_leg;
	delete rr_leg;
}


void LegControl::LegJointControl(void)
{
	robotLegTorque_t torque = {0};
	pthread_mutex_lock(&ref_mutex);

	if (robotModel->lf_state->contact_state == 1)
		torque.lf_torque = lf_leg->SingleStanceControl(robotModel->world2BodyRotation, robotModel->grav.block(6, 0, 3, 1), robotModel->coriolis.block(6, 0, 3, 1), motionPlan->legPlan[0]->fr_des);
	else
		torque.lf_torque = lf_leg->SingleSwingControl(robotModel->world2BodyRotation, robotModel->grav.block(6, 0, 3, 1), robotModel->coriolis.block(6, 0, 3, 1), motionPlan->legPlan[0]->pFoot_des->Pop(), motionPlan->legPlan[0]->vFoot_des->Pop());

	if (robotModel->lr_state->contact_state == 1)
		torque.lr_torque = lr_leg->SingleStanceControl(robotModel->world2BodyRotation, robotModel->grav.block(9, 0, 3, 1), robotModel->coriolis.block(9, 0, 3, 1), motionPlan->legPlan[1]->fr_des);
	else
		torque.lr_torque = lr_leg->SingleSwingControl(robotModel->world2BodyRotation, robotModel->grav.block(9, 0, 3, 1), robotModel->coriolis.block(9, 0, 3, 1), motionPlan->legPlan[1]->pFoot_des->Pop(), motionPlan->legPlan[1]->vFoot_des->Pop());

	if (robotModel->rf_state->contact_state == 1)
		torque.rf_torque = rf_leg->SingleStanceControl(robotModel->world2BodyRotation, robotModel->grav.block(12, 0, 3, 1), robotModel->coriolis.block(12, 0, 3, 1), motionPlan->legPlan[2]->fr_des);
	else
		torque.rf_torque = rf_leg->SingleSwingControl(robotModel->world2BodyRotation, robotModel->grav.block(12, 0, 3, 1), robotModel->coriolis.block(12, 0, 3, 1), motionPlan->legPlan[2]->pFoot_des->Pop(), motionPlan->legPlan[2]->vFoot_des->Pop());

	if (robotModel->rr_state->contact_state == 1)
		torque.rr_torque = rr_leg->SingleStanceControl(robotModel->world2BodyRotation, robotModel->grav.block(15, 0, 3, 1), robotModel->coriolis.block(15, 0, 3, 1), motionPlan->legPlan[3]->fr_des);
	else
		torque.rr_torque = rr_leg->SingleSwingControl(robotModel->world2BodyRotation, robotModel->grav.block(15, 0, 3, 1), robotModel->coriolis.block(15, 0, 3, 1), motionPlan->legPlan[3]->pFoot_des->Pop(), motionPlan->legPlan[3]->vFoot_des->Pop());

	pthread_mutex_unlock(&ref_mutex);

	if (handle.start == 1)
		SetRobotJointTorque(torque);
}