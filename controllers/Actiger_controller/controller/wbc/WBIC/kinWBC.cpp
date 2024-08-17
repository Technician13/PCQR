/**
 * @file     kinWBC.cpp
 * @author   Bin
 * @version  V1.0.1
 * @data     29-March-2021
 * @brief    ���������WBC�˶�ѧ���㲿��
 ******************************************************************************************
 * @attention
 *
 *
**/

#include "kinWBC.hpp"
#include <iostream>

using namespace quadruped;


KinWBC::KinWBC(size_t p_num_qdot)
    :  num_qdot(p_num_qdot), num_act_joint(p_num_qdot - 6)
{
    I_mtx = DMat<double>::Identity(num_qdot, num_qdot);
    std::cout << "KinWBC Init" << std::endl;
}

KinWBC::~KinWBC(void)
{

}

bool KinWBC::FindConfiguration(const DVec<double>& p_curr_config,
	const std::vector<Task*>& p_task_list,
	const std::vector<ContactSpec*>& p_contact_list,
    DVec<double>* p_jpos_cmd, DVec<double>* p_jvel_cmd)
{
    // Contact Jacobian Setup
    DMat<double> Nc(num_qdot, num_qdot); 
    Nc.setIdentity();
    if (p_contact_list.size() > 0) {
        DMat<double> Jc, Jc_i; 
        ContactSpec* contact = p_contact_list[0];

        Jc=contact->GetContactJacobian();
        size_t num_rows = Jc.rows();
        for (size_t i(1); i < p_contact_list.size(); ++i) 
        {
            contact = p_contact_list[i];
            Jc_i = contact->GetContactJacobian();
            size_t num_new_rows = Jc_i.rows(); 
            Jc.conservativeResize(num_rows + num_new_rows, num_qdot);
            Jc.block(num_rows, 0, num_new_rows, num_qdot) = Jc_i;
            num_rows += num_new_rows;
        }
        // Projection Matrix
        BuildProjectionMatrix(Jc, Nc);
    }
    // First Task
    DVec<double> delta_q, qdot;
    DMat<double> Jt, JtPre, JtPre_pinv, N_nx, N_pre;
     
    Task* task = p_task_list[0];
    task->GetTaskJacobian(Jt);
    JtPre = Jt * Nc;
    JtPre_pinv = Common::Pinv(JtPre);

    delta_q = JtPre_pinv * (task->GetPosError());
    qdot = JtPre_pinv * (task->GetDesVel());

    DVec<double> prev_delta_q = delta_q;
    DVec<double> prev_qdot = qdot;
    BuildProjectionMatrix(JtPre, N_nx);
    N_pre = Nc * N_nx;
    for (size_t i(1); i < p_task_list.size(); ++i) {
        task = p_task_list[i];
        task->GetTaskJacobian(Jt);
        JtPre = Jt * N_pre; 
        JtPre_pinv = Common::Pinv(JtPre);
        delta_q =
            prev_delta_q + JtPre_pinv * (task->GetPosError() - Jt * prev_delta_q);
        qdot = prev_qdot + JtPre_pinv * (task->GetDesVel() - Jt * prev_qdot);
        // For the next task
        BuildProjectionMatrix(JtPre, N_nx);
        N_pre *= N_nx;
        prev_delta_q = delta_q;
        prev_qdot = qdot;
    }

    for (size_t i(0); i < num_act_joint; ++i) {
        (*p_jpos_cmd)(i) = p_curr_config(i + 6) + delta_q(i + 6);
        (*p_jvel_cmd)(i) = qdot(i + 6);
        //std::cout << curr_config_(i+6) << " ";
        //std::cout << (*jpos_cmd_)(i) << " ";
        //std::cout << (*jvel_cmd_)(i) << " ";
        //std::cout << std::endl;
    }

    
    return true;
}

void KinWBC::BuildProjectionMatrix(const DMat<double>& p_J, DMat<double>& p_N)
{
    DMat<double> J_pinv;
	J_pinv = Common::Pinv(p_J);
	p_N = I_mtx - J_pinv * p_J;
}


