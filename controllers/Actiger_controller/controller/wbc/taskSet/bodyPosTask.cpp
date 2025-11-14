#include "bodyPosTask.hpp"
#include <iostream>

using namespace quadruped;

BodyPosTask::BodyPosTask(RobotModel* p_robotModel)
	: Task(3), robotModel(p_robotModel)
{
	Task::Jt = DMat<double>::Zero(Task::dim_task, quadruped::dim_config);
	Task::Jt.block(0, 3, 3, 3).setIdentity();
	Task::JtDotQdot = DVec<double>::Zero(Task::dim_task);

	kp_kin = DVec<double>::Constant(Task::dim_task, 1.);
	kp = DVec<double>::Constant(Task::dim_task, 50.);
	kd = DVec<double>::Constant(Task::dim_task, 1.);

	std::cout << "BodyPosTask Init" << std::endl;
}


BodyPosTask::~BodyPosTask(void)
{

}


bool BodyPosTask::UpdateCommand(const DVec<double>& p_pos_des, const DVec<double>& p_vel_des, const DVec<double>& p_acc_des)
{
	Vec3<double> pos_ref = p_pos_des;
	Vec3<double> body_pos = robotModel->bodyPosition;
	Vec3<double> curr_vel = robotModel->bodyVelocity;

	Vec3<double> vel_err = Task::vel_des - curr_vel;
	for (int i(0); i < 3; ++i) {
		Task::pos_err[i] = kp_kin[i] * (pos_ref[i] - body_pos[i]);
		Task::vel_des[i] = p_vel_des[i];
		Task::acc_des[i] = p_acc_des[i];

		Task::acc_cmd[i] = kp[i] * Task::pos_err[i] +
			kd[i] * (vel_err[i]) + Task::acc_des[i];
	}
	return true;
}


bool BodyPosTask::UpdateTaskJacobian(void)
{
	return true;
}


bool BodyPosTask::UpdateTaskJDotQdot(void)
{
	return true;
}

