/**
 * @file     singleContact.cpp
 * @author   Bin
 * @version  V1.0.1
 * @data     29-March-2021
 * @brief    ��������˵����Ӵ�����غ����ļ�
 ******************************************************************************************
 * @attention
 *
 *
**/

#include "singleContact.hpp"

using namespace quadruped;
// [ Fx, Fy, Fz ]

SingleContact::SingleContact(RobotModel* p_robotModel, const int p_linkId)
	:ContactSpec(3), contact_id(p_linkId), dim_cstr(5), robotModel(p_robotModel)
{
	ContactSpec::Jc = DMat<double>::Zero(ContactSpec::dim_contact, dim_config);
	ContactSpec::JcDotQdot = DVec<double>::Zero(ContactSpec::dim_contact);
	ContactSpec::cstrMtx = DMat<double>::Zero(dim_cstr, ContactSpec::dim_contact);

	double mu(0.8);

	ContactSpec::cstrMtx << 1.0, 0, -mu * 0.707,
		                       1.0, 0, mu * 0.707,
		                       0, 1.0, -mu * 0.707,
		                       0, 1.0, mu * 0.707,
		                       0, 0, 1.0;
}


SingleContact::~SingleContact()
{

}


bool SingleContact::UpdateJc() {
	ContactSpec::Jc = robotModel->J[contact_id];

	return true;
}


bool SingleContact::UpdateJcDotQdot() {
	ContactSpec::JcDotQdot = robotModel->Jdqd[contact_id];
	return true;
}


bool SingleContact::UpdateUf() {
	return true;
}


bool SingleContact::UpdateInequalityVector() {
	double bigNum = 1.0e6;
	ContactSpec::ieq_vec_l = DVec<double>::Zero(dim_cstr);
	ContactSpec::ieq_vec_l << -bigNum,
		                          0.0,
		                         -bigNum,
		                          0.0,
		                          5.0;
	ContactSpec::ieq_vec_u = DVec<double>::Zero(dim_cstr);
	ContactSpec::ieq_vec_u << 0.0,
		                         bigNum,
		                         0.0,
		                         bigNum,
		                         300.0;
	return true;
}
