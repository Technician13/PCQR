#include "dynWBC.hpp"
#include <iostream>

using namespace std;
using namespace quadruped;

DynWBC::DynWBC(size_t p_num_qdot,
                  const std::vector<ContactSpec*>* p_contact_list,
				  const std::vector<Task*>* p_task_list)
	: contact_list(p_contact_list), task_list(p_task_list),
    num_qdot(p_num_qdot), num_act_joint(p_num_qdot - 6)
{
    qpSolver = new QpSolver();
    Sv = DMat<double>::Zero(6, p_num_qdot);
    Sv.block(0,  0, 6, 6).setIdentity();
    eye.setIdentity();
    std::cout << "DynWBC Init" << std::endl;
}

DynWBC::~DynWBC()
{

}


DVec<double> DynWBC::MakeTorque(void)
{
    if (!b_updatesetting) {
        printf("[Wanning] WBIC setting is not done\n");
    }
    SetOptimizationSize();

    DVec<double> qddot_pre;
    DMat<double> JcBar;
    DMat<double> Npre;
    if (dim_rf > 0) {
        ContactBuilding();
        JcBar = Common::WeightedInverse(Jc, Ainv);
        qddot_pre = JcBar * (-JcDotQdot);
        Npre = eye - JcBar * Jc;
    }
    else {
        qddot_pre = DVec<double>::Zero(num_qdot);
        Npre = eye;
    }

    Task* task;
    DMat<double> Jt, JtBar, JtPre;
    DVec<double> JtDotQdot, xddot;
    for (size_t i(0); i < (*task_list).size(); ++i) {
        task = (*task_list)[i];

        task->GetTaskJacobian(Jt);

        task->GetTaskJacobianDotQdot(JtDotQdot);
        task->GetCommand(xddot);
        JtPre = Jt * Npre;
        JtBar = Common::WeightedInverse(JtPre, Ainv);
        qddot_pre += JtBar * (xddot - JtDotQdot - Jt * qddot_pre);
        Npre = Npre * (eye - JtBar * JtPre);

    }
    CalculateQPData(qddot_pre);

    DVec<double> add;
    add = qpSolver->Solve(qp_H, qp_G, qp_A, qp_L, qp_U);

    DVec<double> cmd_leg;
    cmd_leg = GetSolution(qddot_pre, add);
    return cmd_leg;
}



void DynWBC::CalculateQPData(const DVec<double>& p_qddot) \
{
    qp_H.block(0, 0, dim_floating, dim_floating) = w_floating;

    if(dim_rf > 0)
        qp_H.block(dim_floating, dim_floating, dim_rf, dim_rf) = w_rf.block(0, 0, dim_rf, dim_rf);

    qp_G.setZero();
    if (dim_rf > 0)
    {
        qp_A.block(0, 0, dim_floating, dim_floating) = A.block(0, 0, dim_floating, dim_floating);
        qp_A.block(0, dim_floating, dim_floating, dim_rf) = -Sv * Jc.transpose();
        qp_A.block(dim_floating, dim_floating, dim_cstrMtx, dim_rf) = cstrMtx;

        qp_U.block(0, 0, dim_floating, 1) = -Sv * (A * p_qddot + cori + grav -
            Jc.transpose() * Fr_des);
        qp_U.block(dim_floating, 0, dim_cstrMtx, 1) = cstr_ieq_vec_u - cstrMtx * Fr_des;

        qp_L.block(0, 0, dim_floating, 1) = -Sv * (A * p_qddot + cori + grav -
            Jc.transpose() * Fr_des);
        qp_L.block(dim_floating, 0, dim_cstrMtx, 1) = cstr_ieq_vec_l - cstrMtx * Fr_des;
    }
    else {

        qp_A.block(0, 0, dim_floating, dim_floating) = A.block(0, 0, dim_floating, dim_floating);

        qp_U = -Sv * (A * p_qddot + cori + grav);

        qp_L = -Sv * (A * p_qddot + cori + grav);

    }
}



void DynWBC::ContactBuilding() 
{
    DMat<double> Uf_block;
    DVec<double> Uf_ieq_vec_l_block;
    DVec<double> Uf_ieq_vec_u_block;
    DMat<double> Jc_block;
    DVec<double> JcDotQdot_block;
    size_t dim_accumul_rf, dim_accumul_uf;
    Jc_block = (*contact_list)[0]->GetContactJacobian();
    JcDotQdot_block = (*contact_list)[0]->GetJcDotQdot();
    Uf_block = (*contact_list)[0]->GetRFConstraintMtx();
    Uf_ieq_vec_u_block = (*contact_list)[0]->GetRFConstraintVecU();
    Uf_ieq_vec_l_block = (*contact_list)[0]->GetRFConstraintVecL();

    dim_accumul_rf = (*contact_list)[0]->GetDim();
    dim_accumul_uf = (*contact_list)[0]->GetDimRFConstraint();

    Jc.block(0, 0, dim_accumul_rf, num_qdot) = Jc_block;
    JcDotQdot.head(dim_accumul_rf) = JcDotQdot_block;
    cstrMtx.block(0, 0, dim_accumul_uf, dim_accumul_rf) = Uf_block;
    cstr_ieq_vec_u.head(dim_accumul_uf) = Uf_ieq_vec_u_block;
    cstr_ieq_vec_l.head(dim_accumul_uf) = Uf_ieq_vec_l_block;
    Fr_des.head(dim_accumul_rf) = (*contact_list)[0]->GetRFDesired();

    size_t dim_new_rf, dim_new_uf;

    for (size_t i(1); i < (*contact_list).size(); ++i) 
    {
        Jc_block = (*contact_list)[i]->GetContactJacobian();
        JcDotQdot_block = (*contact_list)[i]->GetJcDotQdot();

        dim_new_rf = (*contact_list)[i]->GetDim();
        dim_new_uf = (*contact_list)[i]->GetDimRFConstraint();

        Jc.block(dim_accumul_rf, 0, dim_new_rf, num_qdot) = Jc_block;

        JcDotQdot.segment(dim_accumul_rf, dim_new_rf) = JcDotQdot_block;

        Uf_block = (*contact_list)[i]->GetRFConstraintMtx();
        cstrMtx.block(dim_accumul_uf, dim_accumul_rf, dim_new_uf, dim_new_rf) = Uf_block;

        Uf_ieq_vec_l_block =(*contact_list)[i]->GetRFConstraintVecL();
        cstr_ieq_vec_l.segment(dim_accumul_uf, dim_new_uf) = Uf_ieq_vec_l_block;

        Uf_ieq_vec_u_block = (*contact_list)[i]->GetRFConstraintVecU();
        cstr_ieq_vec_u.segment(dim_accumul_uf, dim_new_uf) = Uf_ieq_vec_u_block;

        Fr_des.segment(dim_accumul_rf, dim_new_rf) =
            (*contact_list)[i]->GetRFDesired();
        dim_accumul_rf += dim_new_rf;
        dim_accumul_uf += dim_new_uf;
    }
}

DVec<double> DynWBC::GetSolution(const DVec<double>& p_qddot, DVec<double> p_add)
{
    DVec<double> cmd_leg;
    DVec<double> tot_tau;

    Vec18<double> qddot = Vec18<double>::Zero();

    qddot = p_qddot;
    qddot.block(0, 0, dim_floating, 1) = 
        qddot.block(0, 0, dim_floating, 1) + p_add.block(0, 0, dim_floating, 1);
    if (dim_rf > 0) {
        Fr = DVec<double>::Zero(dim_rf);
        for (size_t i(0); i < dim_rf; ++i)
            Fr[i] = p_add[i + dim_floating] + Fr_des[i];
        tot_tau =
            A * qddot + cori + grav - Jc.transpose() * Fr;
    }
    else 
    {
        tot_tau = A * qddot + cori + grav;
    }
    
    cmd_leg = tot_tau.tail(num_act_joint);
    return cmd_leg;
}


void DynWBC::UpdateSetting(const DMat<double>& p_A, const DMat<double>& p_Ainv,
    const DVec<double>& p_cori, const DVec<double>& p_grav)
{
    A = p_A;
    Ainv = p_Ainv;
    cori = p_cori;
    grav = p_grav;
    b_updatesetting = true;
}

void DynWBC::SetOptimizationSize() 
{
    dim_rf = 0;
    dim_cstrMtx = 0;  
    for (size_t i(0); i < (*contact_list).size(); ++i) {
        dim_rf += (*contact_list)[i]->GetDim();
        dim_cstrMtx += (*contact_list)[i]->GetDimRFConstraint();
    }

    dim_opt = dim_floating + dim_rf;

    qp_H.resize(dim_opt, dim_opt);
    qp_G.resize(dim_opt);
   

    if (dim_rf > 0) {
        qp_A.resize(dim_floating + dim_cstrMtx, dim_opt);
        qp_U.resize(dim_floating + dim_cstrMtx);
        qp_L.resize(dim_floating + dim_cstrMtx);

        Jc = DMat<double>::Zero(dim_rf, num_qdot);
        JcDotQdot = DVec<double>(dim_rf);
        Fr_des = DVec<double>(dim_rf);

        cstrMtx = DMat<double>(dim_cstrMtx, dim_rf);
        cstrMtx.setZero();
        cstr_ieq_vec_l = DVec<double>(dim_cstrMtx);
        cstr_ieq_vec_l.setZero();
        cstr_ieq_vec_u = DVec<double>(dim_cstrMtx);
        cstr_ieq_vec_u.setZero();
    }
    else {
        qp_A.resize(dim_floating, dim_opt);
        qp_U.resize(dim_floating);
        qp_L.resize(dim_floating);
    }
    qp_H.setZero();
    qp_G.setZero();
    qp_A.setZero();
    qp_U.setZero();
    qp_L.setZero();

}
