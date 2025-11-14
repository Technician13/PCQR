#include "bodyOriTask.hpp"
#include <iostream>

using namespace quadruped;


BodyOriTask::BodyOriTask(RobotModel* p_robotModel)
	: Task(3), robotModel(p_robotModel)
{
	Task::Jt = DMat<double>::Zero(Task::dim_task, dim_config);
	Task::Jt.block(0, 0, 3, 3).setIdentity();
	Task::JtDotQdot = DVec<double>::Zero(Task::dim_task);

	std::cout << "BodyOriTask Init" << std::endl;
}


BodyOriTask::~BodyOriTask(void)
{

}


bool BodyOriTask::UpdateCommand(const DVec<double>& p_pos_des, const DVec<double>& p_vel_des,
			const DVec<double>& p_acc_des)
{
	Vec3<double> ori_ref = p_pos_des;
	Vec3<double> ori_pos = robotModel->bodyEuler;

	Vec3<double> curr_vel = robotModel->bodyEulerAngularVel;

	Vec3<double> vel_err = (Task::vel_des - curr_vel);

	for (int i(0); i < 3; ++i) {
		Task::pos_err(i) = kp_kin(i) * (ori_ref(i) - ori_pos(i));
		Task::vel_des(i) = p_vel_des(i);
		Task::acc_des(i) = p_acc_des(i);

		Task::acc_cmd(i) =kp(i) * Task::pos_err(i) +
			kd(i) * vel_err(i) + Task::acc_des(i);
	}
	return true;
}


bool BodyOriTask::UpdateTaskJacobian(void)
{
	return true;
}


bool BodyOriTask::UpdateTaskJDotQdot(void)
{
  	return true;
}

