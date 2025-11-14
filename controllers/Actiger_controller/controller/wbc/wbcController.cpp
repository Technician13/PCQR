#include "wbcController.hpp"
#include <iostream>

using namespace quadruped;

WBC_Ctrl::WBC_Ctrl(RobotModel* p_robotModel, MotionPlan* p_ref, Parameters* p_para, LegControl* p_legControl)
{
	robotModel = p_robotModel;
	ref = p_ref;
	para = p_para;
	legControl = p_legControl;
	body_pos_task = new BodyPosTask(robotModel);
	body_ori_task = new BodyOriTask(robotModel);
	  
	foot_contact[0] = new SingleContact(robotModel, LF);
	foot_contact[1] = new SingleContact(robotModel, LR);
	foot_contact[2] = new SingleContact(robotModel, RF);
	foot_contact[3] = new SingleContact(robotModel, RR);

	link_task[0] = new LinkPosTask(robotModel, LF);
	link_task[1] = new LinkPosTask(robotModel, LR);
	link_task[2] = new LinkPosTask(robotModel, RF);
	link_task[3] = new LinkPosTask(robotModel, RR);
	 
	kin_wbc = new KinWBC(dim_config);
	dyn_wbc = new DynWBC(dim_config,&contact_list, &task_list);

	ParameterSetup();
	std::cout << "WBC_Ctrl Init" << std::endl;
}


WBC_Ctrl::~WBC_Ctrl(void)
{
	delete body_pos_task;
	delete body_ori_task;

	for (size_t i(0); i < 4; ++i) {
		delete foot_contact[i];
		delete link_task[i];
	}

	delete kin_wbc;
	delete dyn_wbc;

	typename std::vector<Task*>::iterator iter = task_list.begin();
	while (iter < task_list.end()) {
		delete (*iter);
		++iter;
	}
	task_list.clear();

	typename std::vector<ContactSpec*>::iterator iter2 = contact_list.begin();
	while (iter2 < contact_list.end()) {
		delete (*iter2);
		++iter2;
	}
	contact_list.clear();
}


void WBC_Ctrl::ParameterSetup(void)
{
	dyn_wbc->w_floating = para->wbc_w_floating;
	dyn_wbc->w_rf = para->wbc_w_rf;

	body_pos_task->kp = para->wbc_kp_pos;
	body_pos_task->kd = para->wbc_kd_pos;
	body_pos_task->kp_kin = para->wbc_kp_kin_pos;

	body_ori_task->kp = para->wbc_kp_ori;
	body_ori_task->kd = para->wbc_kd_ori;
	body_ori_task->kp_kin = para->wbc_kp_kin_ori;

	for (size_t j(0); j < 4; ++j) 
	{
		link_task[j]->kp = para->wbc_kp_link;
		link_task[j]->kd = para->wbc_kd_link;
		link_task[j]->kp_kin = para->wbc_kp_kin_link;
	}
	
}


void WBC_Ctrl::UpdateLegCMD()
{
	for(int i=0;i<num_leg_joint;i++)
		legControl->lf_leg->torqueFeedForward(i)=torque_ff(i);

	for(int i=0;i<num_leg_joint;i++)
		legControl->lf_leg->qDes(i)=des_jpos(i);

	for(int i=0;i<num_leg_joint;i++)
		legControl->lf_leg->qdDes(i)=des_jvel(i);

	for(int i=0;i<num_leg_joint;i++)
		legControl->lr_leg->torqueFeedForward(i)=torque_ff(3*1+i);

	for(int i=0;i<num_leg_joint;i++)
		legControl->lr_leg->qDes(i)=des_jpos(3*1+i);

	for(int i=0;i<num_leg_joint;i++)
		legControl->lr_leg->qdDes(i)=des_jvel(3*1+i);

	for(int i=0;i<num_leg_joint;i++)
		legControl->rf_leg->torqueFeedForward(i)=torque_ff(3*2+i);

	for(int i=0;i<num_leg_joint;i++)
		legControl->rf_leg->qDes(i)=des_jpos(3*2+i);

	for(int i=0;i<num_leg_joint;i++)
		legControl->rf_leg->qdDes(i)=des_jvel(3*2+i);

	for(int i=0;i<num_leg_joint;i++)
		legControl->rr_leg->torqueFeedForward(i)=torque_ff(3*3+i);

	for(int i=0;i<num_leg_joint;i++)
		legControl->rr_leg->qDes(i)=des_jpos(3*3+i);

	for(int i=0;i<num_leg_joint;i++)
		legControl->rr_leg->qdDes(i)=des_jvel(3*3+i);
}


void WBC_Ctrl::CleanUp() {
	contact_list.clear();
	task_list.clear();
}


void WBC_Ctrl::UpdatTask(void)
{
	CleanUp();

	Vec3<double> zero_vec3= Vec3<double>::Zero();

	pthread_mutex_lock(&ref_mutex);
	body_ori_task->UpdateTask(ref->pBody_euler_des->Pop(), ref->vBody_euler_des->Pop(), zero_vec3);
	body_pos_task->UpdateTask(ref->pBody_des->Pop(), ref->vBody_des->Pop(), ref->aBody_des->Pop());
	pthread_mutex_unlock(&ref_mutex);
	
	task_list.push_back(body_ori_task);
	task_list.push_back(body_pos_task);

	for (size_t leg(0); leg < 4; ++leg) 
	{
		if(robotModel->legState[leg]->contact_state == 1) 
		{ 
			foot_contact[leg]->SetRFDesired(ref->legPlan[leg]->fr_des);
			foot_contact[leg]->UpdateContactSpec();
			contact_list.push_back(foot_contact[leg]);
		}
		else { 
			link_task[leg]->UpdateTask((ref->legPlan[leg]->pFoot_des->Pop()), ref->legPlan[leg]->vFoot_des->Pop(), ref->legPlan[leg]->aFoot_des->Pop());
			task_list.push_back(link_task[leg]);
		}
	}	
}


void WBC_Ctrl::ComputeWBC(void)
{
	des_jpos.resize(dim_config - 6, 1);
	des_jpos.setZero();
	des_jvel.resize(dim_config - 6, 1);
	des_jvel.setZero();

	full_config = robotModel->state_now;
	kin_wbc->FindConfiguration(full_config, task_list, contact_list,
		&des_jpos, &des_jvel);

	dyn_wbc->UpdateSetting(robotModel->A, robotModel->Ainv, robotModel->coriolis, robotModel->grav);
	torque_ff=dyn_wbc->MakeTorque();
}


void WBC_Ctrl::Run(Vec12<double> & p_mpc_force)
{
	pthread_mutex_lock(&f_des_mutex);
	for(int leg(0);leg<4;leg++)
	{
		p_mpc_force.block( leg*3, 0, 3, 1)=ref->legPlan[leg]->fr_des;
	}
	UpdatTask();
	pthread_mutex_unlock(&f_des_mutex);
 	
	ComputeWBC();

	UpdateLegCMD();

}

