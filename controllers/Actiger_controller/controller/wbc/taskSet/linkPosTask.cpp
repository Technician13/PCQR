/**
 * @file     linkPosTask.cpp
 * @author   Bin
 * @version  V1.0.1
 * @data     29-March-2021
 * @brief    ��������˵�������
 ******************************************************************************************
 * @attention
 *
 *
**/


#include "linkPosTask.hpp"
#include <iostream>

using namespace quadruped;


LinkPosTask::LinkPosTask(RobotModel* p_robotModel, const int p_linkId)
	: Task(3),
	robotModel(p_robotModel), 
	linkId(p_linkId)
{
	Task::Jt = DMat<double>::Zero(Task::dim_task, quadruped::dim_config);
	Task::JtDotQdot = DVec<double>::Zero(Task::dim_task);

	std::cout << "LinkPosTask Init" << std::endl;
}

LinkPosTask::~LinkPosTask(void)
{

}

bool LinkPosTask::UpdateCommand(const DVec<double>& p_pos_des, const DVec<double>& p_vel_des, const DVec<double>& p_acc_des)
{
	Vec3<double> pos_ref = (Vec3<double>)p_pos_des;
	Vec3<double> link_pos;

	link_pos = robotModel->linkEndPos[linkId];

	// X, Y, Z
	for (int i(0); i < 3; ++i) {
		Task::pos_err[i] = kp_kin[i] * ((pos_ref)[i] - link_pos[i]);
		Task::vel_des[i] = p_vel_des[i];
		Task::acc_des[i] = p_acc_des[i];
		
		// std::cout << link_pos[i] << " ";
	}
	
	//std::cout << "pos_err" << std::endl;
	//std::cout << Task::pos_err << std::endl;

	// Op acceleration command
	for (size_t i(0); i < Task::dim_task; ++i) {
		Task::acc_cmd[i] =
			kp[i] * Task::pos_err[i] +
			kd[i] * (Task::vel_des[i] - robotModel->linkEndVel[linkId][i]) +
			Task::acc_des[i];
	}
	//std::cout << "acc_cmd" << std::endl;
	//std::cout << Task::acc_cmd << std::endl;
	return true;
	 
}

bool LinkPosTask::UpdateTaskJacobian(void) 
{
	Task::Jt = robotModel->J[linkId];
	Task::Jt.block(0, 0, 3, 6) = DMat<double>::Zero(3, 6);
	return true;

}

bool LinkPosTask::UpdateTaskJDotQdot(void)
{
	Task::JtDotQdot = robotModel->Jdqd[linkId];
	return true;

}
