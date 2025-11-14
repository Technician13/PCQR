#include <iostream>
#include "VelocityEstimate.hpp"

VelocityEstimate::VelocityEstimate(void)
{
    double LF_probabilistic = 0.0;
	double LR_probabilistic = 0.0;
	double RF_probabilistic = 0.0;
	double RR_probabilistic = 0.0;

    b_xdot_lf = Vec3<double>::Zero(3);
	b_xdot_lr = Vec3<double>::Zero(3);
	b_xdot_rf = Vec3<double>::Zero(3);
	b_xdot_rr = Vec3<double>::Zero(3);

    v_k_1 = Vec3<double>::Zero(3);
    xdot = Vec3<double>::Zero(3);
    p_k = Vec3<double>::Zero(3);
    p_k_1 = Vec3<double>::Zero(3);
    a_k_1 = Vec3<double>::Zero(3);
    a_k = Vec3<double>::Zero(3);

    KalmanCnt = 0;
    beta = 1.0;
    beta0 = 0.0;

	LF_CONTACT_LAST = 1;
    LR_CONTACT_LAST = 1;
    RF_CONTACT_LAST = 1;
    RR_CONTACT_LAST = 1;

	period = TIME_STEP;
}


VelocityEstimate::~VelocityEstimate()
{
    delete KF;
}

void VelocityEstimate::vector2crossmatrix(const Vec3<double> & vector_input, Mat3<double> & matrix_input)
{
    matrix_input.setZero();
	matrix_input(0, 1) = -vector_input(2);
	matrix_input(0, 2) = vector_input(1);
	matrix_input(1, 0) = vector_input(2);
	matrix_input(1, 2) = -vector_input(0);
	matrix_input(2, 0) = -vector_input(1);
	matrix_input(2, 1) = vector_input(0);
}

Vec3<double> VelocityEstimate::CalVelocity(bool LF_CONTACT, bool LR_CONTACT, bool RF_CONTACT, bool RR_CONTACT,
                                		   Vec3<double> lf_endfoot_vel, Vec3<double> lr_endfoot_vel, Vec3<double> rf_endfoot_vel, Vec3<double> rr_endfoot_vel,
                                		   Vec3<double> lf_endfoot, Vec3<double> lr_endfoot, Vec3<double> rf_endfoot, Vec3<double> rr_endfoot,
                                		   DVec<double> f_k, DVec<double> f_k_1, Mat3<double> rotation,
                                		   Vec3<double> actul_body_angularVel, Vec3<double> actul_body_acc, Vec3<double> actul_body_pos)
{
    (LF_CONTACT_LAST == 1 && LF_CONTACT == 1) ? contactList[0] = 1 : contactList[0] = 0;
	(LR_CONTACT_LAST == 1 && LR_CONTACT == 1) ? contactList[1] = 1 : contactList[1] = 0;
	(RF_CONTACT_LAST == 1 && RF_CONTACT == 1) ? contactList[2] = 1 : contactList[2] = 0;
	(RR_CONTACT_LAST == 1 && RR_CONTACT == 1) ? contactList[3] = 1 : contactList[3] = 0;
    contactNum = 0;
    for (std::vector<bool>::iterator iter = contactList.begin(); iter != contactList.end(); ++iter)
	{
		if (1 == *iter)
		{
			contactNum += 1;
		}
	}
    vector2crossmatrix(actul_body_angularVel, omega_cross);
	
	if (contactList[0])
	{
		b_xdot_lf = -(lf_endfoot_vel) - omega_cross * (lf_endfoot);
		LF_probabilistic = 1.0 / (1 + exp(-beta * (f_k(2)) - beta0));
		b_xdot_lf = b_xdot_lf * LF_probabilistic;
	}
	else
	{
		b_xdot_lf = Vec3<double>::Zero(3);
		LF_probabilistic = 0.0;
	}
	if (contactList[1])
	{
		b_xdot_lr = -(lr_endfoot_vel) - omega_cross * (lr_endfoot);
		LR_probabilistic = 1.0 / (1 + exp(-beta * (f_k(5)) - beta0));
		b_xdot_lr = b_xdot_lr * LR_probabilistic;
	}
	else
	{
		b_xdot_lr = Vec3<double>::Zero(3);
		LR_probabilistic = 0.0;
	}
	if (contactList[2])
	{
		b_xdot_rf = -(rf_endfoot_vel) - omega_cross * (rf_endfoot);
		RF_probabilistic = 1.0 / (1 + exp(-beta * (f_k(8)) - beta0));
		b_xdot_rf = b_xdot_rf * RF_probabilistic;
	}
	else
	{
		b_xdot_rf = Vec3<double>::Zero(3);
		RF_probabilistic = 0.0;
	}
	if (contactList[3])
	{
		b_xdot_rr = -(rr_endfoot_vel) - omega_cross * (rr_endfoot);
		RR_probabilistic = 1.0 / (1 + exp(-beta * (f_k(11)) - beta0));
		b_xdot_rr = b_xdot_rr*RR_probabilistic;
	}
	else
	{
		b_xdot_rr = Vec3<double>::Zero(3);
		RR_probabilistic = 0.0;
	}
	
    velocityPrimary = (b_xdot_lf + b_xdot_lr + b_xdot_rf + b_xdot_rr) / (LF_probabilistic + LR_probabilistic + RF_probabilistic + RR_probabilistic);
	world_velocityPrimary = rotation.inverse() * velocityPrimary;

    a_k = actul_body_acc;
    a_k = rotation.inverse()*a_k;
    a_k_1;

    if (KalmanCnt>5)
	{
		p_k = actul_body_pos;
		xdot = world_velocityPrimary;

		KF->Init(v_k_1, f_k, f_k_1, xdot, p_k, p_k_1, a_k_1);
		KF->Ready(contactList, contactNum, b_xdot_lf, b_xdot_lr, b_xdot_rf, b_xdot_rr);
		KF->Filter();

		p_k_1 = (KF->Result()).block(0, 0, 3, 1);
		v_k_1 = (KF->Result()).block(3, 0, 3, 1);

		a_k_1 = a_k;
	}
	else
	{
		KalmanCnt++;
        v_k_1 = world_velocityPrimary;
	}

	LF_CONTACT_LAST = LF_CONTACT;
    LR_CONTACT_LAST = LR_CONTACT;
    RF_CONTACT_LAST = RF_CONTACT;
    RR_CONTACT_LAST = RR_CONTACT;

    return v_k_1;
}
