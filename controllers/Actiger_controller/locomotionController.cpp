/**
 * @file locomotionController.cpp
 * @brief Actiger运动控制类LocomotionCtrl
 * @author Bin & Tech13
 * @version 1.1
 * @date 2021-08-18
 * 
 * @copyright  Copyright  (c)  2021  Action
 * 
 */

#include "locomotionController.hpp"
#include <iostream>

using namespace quadruped;

/**
 * @brief LocomotionCtrl类的构造函数，完成所有对象成员的内存分配
 */
LocomotionCtrl::LocomotionCtrl(void)
{
	para = new Parameters();
	robotModel = new RobotModel(para);
	motionPlan = new MotionPlan(robotModel, para);
	legControl = new LegControl(robotModel, motionPlan, para);
	mpc = new Mpc(robotModel,motionPlan,para);

	std::cout << "LocomotionCtrl Init" << std::endl;
}

/**
 * @brief LocomotionCtrl类的析构函数，释放申请的内存
 */
LocomotionCtrl::~LocomotionCtrl(void)
{
	delete para;
	delete robotModel;
	delete motionPlan;
	delete legControl;
	delete mpc;
}

/**
 * @brief 机器人控制函数
 */
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