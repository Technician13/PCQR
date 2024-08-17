/**
 * @file robotModel.cpp
 * @brief  Actiger机体模型类RobotModel
 * @author Bin & Tech13
 * @version 1.1
 * @date 2021-08-19
 * 
 * @copyright  Copyright  (c)  2021  Action
 * 
 */

#include "robotModel.hpp"
#include <iostream>

using namespace quadruped;

/**
 * @brief RobotModel类的构造函数，完成所有对象成员的内存分配以及数据成员的初值装载
 * @param  p_para  Parameters类的实例化对象，完成本地中髋关节在机体坐标系下的位置的初始化        
 */
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

/**
 * @brief RobotModel类的析构函数，释放申请的内存
 */
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

/**
 * @brief 部分数据成员重置
 */
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

/**
 * @brief 空函数  |  预留接口  |  获取当前足端在机体坐标系下的位置
 * @return Vec3<double> 
 */
Vec3<double> RobotModel::CalEndfootPos2Body(void) {}

/**
 * @brief 空函数  |  预留接口  |  获取当前足端在机体坐标系下的速度
 * @return Vec3<double> 
 */
Vec3<double> RobotModel::CalEndfootVel2Body(void) {}

/**
 * @brief 机器人状态更新
 * @param  p_mpc_force  上一时刻经过Actiger模型预测控制器计算得到的足端力    
 * @param  p_mpc_force_1  上上时刻经过Actiger模型预测控制器计算得到的足端力
 */
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

	/* --------------------------------------- 目前速度和位置估计不好用 --------------------------------------- */
	// GetRobotBodyAcc(bodyAcc);
	// //std::cout<<"ACCACCACCACCACCACCACCACCACCACCACCACCACCACC : "<<bodyAcc[0]<<"    "<<bodyAcc[1]<<"    "<<bodyAcc[2]<<std::endl;
	// pos_vel_estimator->Run(bodyEuler , bodyAcc , bodyEulerAngularVel ,
	// 					   legState[0]->hip_pos_body , legState[1]->hip_pos_body , legState[2]->hip_pos_body , legState[3]->hip_pos_body ,
	// 					   legState[0]->endfoot_hip , legState[1]->endfoot_hip , legState[2]->endfoot_hip , legState[3]->endfoot_hip ,
	// 					   legState[0]->endfoot_vel_hip , legState[1]->endfoot_vel_hip , legState[2]->endfoot_vel_hip ,legState[3]->endfoot_vel_hip ,
	// 					   legState[0]->contact_state , legState[1]->contact_state , legState[2]->contact_state , legState[3]->contact_state);
	
	Vec3<double> pos, vel;
	GetRobotBodyVel(pos, vel);
	
	// std::cout<<"Estimate : "<<pos_vel_estimator->xhat[0]<<"    "
	// 					    <<pos_vel_estimator->xhat[1]<<"    "
	// 						<<pos_vel_estimator->xhat[2]<<"    "
	// 						<<pos_vel_estimator->xhat[3]<<"    "
	// 						<<pos_vel_estimator->xhat[4]<<"    "
	// 						<<pos_vel_estimator->xhat[5]<<"    ";
	// std::cout<<"Real : "<<pos[0]<<"    "
	// 					<<pos[1]<<"    "
	// 					<<pos[2]<<"    "
	// 					<<vel[0]<<"    "
	// 					<<vel[1]<<"    "
	// 					<<vel[2]<<"    ";
	

	// bodyState->actul_body_pos = 1.1 * pos_vel_estimator->xhat.block(0,0,3,1);
	// bodyState->actul_body_vel = pos_vel_estimator->xhat.block(3,0,3,1);

	// std::cout<<"actual : "<<bodyState->actul_body_pos[0]<<std::endl;
	bodyState->actul_body_pos = pos;
	bodyState->actul_body_vel = vel;
	/* ---------------------------------------------------------------------------------------------------- */

	bodyEuler = bodyState->actul_body_angle;
	bodyEulerAngularVel = bodyState->actul_body_angularVel;
	bodyPosition = bodyState->actul_body_pos;
	bodyVelocity = bodyState->actul_body_vel;

	// std::cout << " ///////////////////////////// Euler : " << bodyEuler(0) << " " << bodyEuler(1) << " " << bodyEuler(2) << " "
	// 		  << bodyPosition(0) << " " << bodyPosition(1) << " " << bodyPosition(2) << " "
	// 		  << bodyEulerAngularVel(0) << " " << bodyEulerAngularVel(1) << " " << bodyEulerAngularVel(2) << " "
	// 		  << bodyVelocity(0) << " " << bodyVelocity(1) << " " << bodyVelocity(2) << std::endl;

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

	/* 调用RBDL接口函数完成Actiger系统动力学模型参数的配置
	 * 此处使用二次封装后的RBDL，链接：https://zhuanlan.zhihu.com/p/375937799 */
	model->UpdateSystem(q_now, q_dot_now);
	model->getWorld2BodyMatrix(world2BodyRotation);
	body2WorldRotation = dynacore::RotZ_Matrix(bodyEuler[2]) * dynacore::RotY_Matrix(bodyEuler[1]) * dynacore::RotX_Matrix(bodyEuler[0]);

	/* 获取足端在惯性坐标系下的位置和速度 */
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

	/* 获取基本雅可比矩阵（Basic Jacobian） 
	 * 6D state: cols(0-2) is angularVel ;cols(3-5) is vel */
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

	/* 拉格朗日动力学中的惯性项 */
	cenInertia = A.block(0, 0, 6, 6);

	/* 拉格朗日动力学中的离心力-哥氏力项 */
	DVec<double> coriolis_;
	model->getCoriolis(coriolis_);
	coriolis = coriolis_;

	/* 拉格朗日动力学中的重力项 */
	DVec<double> grav_;
	model->getGravity(grav_);
	grav = grav_;
	pthread_mutex_unlock(&model_mutex);
}