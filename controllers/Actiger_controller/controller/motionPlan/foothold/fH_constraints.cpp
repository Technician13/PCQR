#include "fH_constraints.hpp"

using namespace quadruped;

FH_constraints::FH_constraints(FH_updataInfo *p_fH_updataInfo)
{
	updataInfo = p_fH_updataInfo;
}

FH_constraints::~FH_constraints(void) {}


void FH_constraints::GetKinematicsCst(Vec18<double> *p_cst, Vec3<double> p_bodyPos, Vec3<double> p_bodyVel, Vec3<double> p_foothold_front, Vec3<double> p_foothold_rear)
{
	Vec3<double> stance_front_pos_hip = Vec3<double>::Zero();
	Vec3<double> stance_rear_pos_hip = Vec3<double>::Zero();

	Vec3<double> front_pos_hip = Vec3<double>::Zero();
	Vec3<double> rear_pos_hip = Vec3<double>::Zero();

	Vec3<double> pos = p_bodyPos;

	Mat3<double> rotation = Common::RotationOfZMatrix(updataInfo->desire_yaw);

	Vec3<double> front_pos_body = rotation * (p_foothold_front - pos);
	Vec3<double> rear_pos_body = rotation * (p_foothold_rear - pos);
	Vec3<double> stance_front_pos_body = rotation * (updataInfo->stance_front_pos - pos);
	Vec3<double> stance_rear_pos_body = rotation * (updataInfo->stance_rear_pos - pos);

	if (updataInfo->is_lf_rr_stand)
	{
		stance_front_pos_hip = stance_front_pos_body - updataInfo->lf_hip_pos_body;
		stance_rear_pos_hip = stance_rear_pos_body - updataInfo->rr_hip_pos_body;

		front_pos_hip = front_pos_body - updataInfo->rf_hip_pos_body;
		rear_pos_hip = rear_pos_body - updataInfo->lr_hip_pos_body;
	}
	else
	{
		stance_front_pos_hip = stance_front_pos_body - updataInfo->rf_hip_pos_body;
		stance_rear_pos_hip = stance_rear_pos_body - updataInfo->lr_hip_pos_body;

		front_pos_hip = front_pos_body - updataInfo->lf_hip_pos_body;
		rear_pos_hip = rear_pos_body - updataInfo->rr_hip_pos_body;
	}

	(*p_cst).setZero();

	(*p_cst)(0) = front_pos_hip(0);

	(*p_cst)(1) = front_pos_hip(1) + 3.556 * front_pos_hip(2);
	(*p_cst)(2) = -front_pos_hip(1) + 3.556 * front_pos_hip(2);

	(*p_cst)(3) = front_pos_hip(2);

	(*p_cst)(4) = rear_pos_hip(0);

	(*p_cst)(5) = rear_pos_hip(1) + 3.556 * rear_pos_hip(2);
	(*p_cst)(6) = -rear_pos_hip(1) + 3.556 * rear_pos_hip(2);

	(*p_cst)(7) = rear_pos_hip(2);

	(*p_cst)(8) = front_pos_body(0) - rear_pos_body(0);
	(*p_cst)(9) = front_pos_body(1) - rear_pos_body(1);

	(*p_cst)(10) = stance_front_pos_hip(0);

	(*p_cst)(11) = stance_front_pos_hip(1) + 3.556 * stance_front_pos_hip(2);
	(*p_cst)(12) = -stance_front_pos_hip(1) + 3.556 * stance_front_pos_hip(2);

	(*p_cst)(13) = stance_front_pos_hip(2);

	(*p_cst)(14) = stance_rear_pos_hip(0);

	(*p_cst)(15) = stance_rear_pos_hip(1) + 3.556 * stance_rear_pos_hip(2);
	(*p_cst)(16) = -stance_rear_pos_hip(1) + 3.556 * stance_rear_pos_hip(2);

	(*p_cst)(17) = stance_rear_pos_hip(2);
}


void FH_constraints::GetKinematicsCstBound(double *ub, double *lb)
{

	ub[0] = (double)0.35;
	lb[0] = (double)-0.35;

	ub[3] = (double)-0.2;
	lb[3] = (double)-0.55;

	ub[4] = (double)0.35;
	lb[4] = (double)-0.35;

	ub[7] = (double)-0.2;
	lb[7] = (double)-0.55;

	ub[8] = (double)100000.0;
	lb[8] = (double)0.0;

	if (updataInfo->is_lf_rr_stand)
	{
		lb[9] = -(double)100000.0;
		ub[9] = (double)0.0;
	}
	else
	{
		ub[9] = (double)100000.0;
		lb[9] = (double)0.0;
	}

	if (updataInfo->is_lf_rr_stand)
	{
		ub[1] = -0.096;
		lb[1] = -100000.0;

		ub[2] = 0.096;
		lb[2] = -100000.0;

		ub[5] = 0.096;
		lb[5] = -100000.0;

		ub[6] = -0.096;
		lb[6] = -100000.0;
	}
	else
	{
		ub[1] = 0.096;
		lb[1] = -100000.0;

		ub[2] = -0.096;
		lb[2] = -100000.0;

		ub[5] = -0.096;
		lb[5] = -100000.0;

		ub[6] = 0.096;
		lb[6] = -100000.0;
	}

	for (int i = 0; i < 2; i++)
	{
		ub[i * 4 + 10] = 0.35;
		lb[i * 4 + 10] = -0.35;

		ub[i * 4 + 3 + 10] = -0.2;
		lb[i * 4 + 3 + 10] = -0.55;
	}

	if (updataInfo->is_lf_rr_stand)
	{
		ub[1 + 10] = 0.096;
		lb[1 + 10] = -100000.0;

		ub[2 + 10] = -0.096;
		lb[2 + 10] = -100000.0;

		ub[5 + 10] = -0.096;
		lb[5 + 10] = -100000.0;

		ub[6 + 10] = 0.096;
		lb[6 + 10] = -100000.0;
	}
	else
	{
		ub[1 + 10] = -0.096;
		lb[1 + 10] = -100000.0;

		ub[2 + 10] = 0.096;
		lb[2 + 10] = -100000.0;

		ub[5 + 10] = 0.096;
		lb[5 + 10] = -100000.0;

		ub[6 + 10] = -0.096;
		lb[6 + 10] = -100000.0;
	}
}

void FH_constraints::GetKinematicsCstJacobian(Mat18x13<double> *p_cst_jacobian_value)
{
	(*p_cst_jacobian_value).setZero();
	//前腿
	(*p_cst_jacobian_value)(0, 0) = -cos(updataInfo->desire_yaw);
	(*p_cst_jacobian_value)(1, 0) = sin(updataInfo->desire_yaw);
	(*p_cst_jacobian_value)(2, 0) = -sin(updataInfo->desire_yaw);

	(*p_cst_jacobian_value)(4, 0) = -cos(updataInfo->desire_yaw);
	(*p_cst_jacobian_value)(5, 0) = sin(updataInfo->desire_yaw);
	(*p_cst_jacobian_value)(6, 0) = -sin(updataInfo->desire_yaw);

	(*p_cst_jacobian_value)(10, 0) = -cos(updataInfo->desire_yaw);
	(*p_cst_jacobian_value)(11, 0) = sin(updataInfo->desire_yaw);
	(*p_cst_jacobian_value)(12, 0) = -sin(updataInfo->desire_yaw);

	(*p_cst_jacobian_value)(14, 0) = -cos(updataInfo->desire_yaw);
	(*p_cst_jacobian_value)(15, 0) = sin(updataInfo->desire_yaw);
	(*p_cst_jacobian_value)(16, 0) = -sin(updataInfo->desire_yaw);

	(*p_cst_jacobian_value)(0, 1) = -sin(updataInfo->desire_yaw);
	(*p_cst_jacobian_value)(1, 1) = -cos(updataInfo->desire_yaw);
	(*p_cst_jacobian_value)(2, 1) = cos(updataInfo->desire_yaw);

	(*p_cst_jacobian_value)(4, 1) = -sin(updataInfo->desire_yaw);
	(*p_cst_jacobian_value)(5, 1) = -cos(updataInfo->desire_yaw);
	(*p_cst_jacobian_value)(6, 1) = cos(updataInfo->desire_yaw);

	(*p_cst_jacobian_value)(10, 1) = -sin(updataInfo->desire_yaw);
	(*p_cst_jacobian_value)(11, 1) = -cos(updataInfo->desire_yaw);
	(*p_cst_jacobian_value)(12, 1) = cos(updataInfo->desire_yaw);

	(*p_cst_jacobian_value)(14, 1) = -sin(updataInfo->desire_yaw);
	(*p_cst_jacobian_value)(15, 1) = -cos(updataInfo->desire_yaw);
	(*p_cst_jacobian_value)(16, 1) = cos(updataInfo->desire_yaw);

	(*p_cst_jacobian_value)(1, 2) = -889.0 / 250.0;
	(*p_cst_jacobian_value)(2, 2) = -889.0 / 250.0;
	(*p_cst_jacobian_value)(3, 2) = -1.0;

	(*p_cst_jacobian_value)(5, 2) = -889.0 / 250.0;
	(*p_cst_jacobian_value)(6, 2) = -889.0 / 250.0;
	(*p_cst_jacobian_value)(7, 2) = -1.0;

	(*p_cst_jacobian_value)(11, 2) = -889.0 / 250.0;
	(*p_cst_jacobian_value)(12, 2) = -889.0 / 250.0;
	(*p_cst_jacobian_value)(13, 2) = -1.0;

	(*p_cst_jacobian_value)(15, 2) = -889.0 / 250.0;
	(*p_cst_jacobian_value)(16, 2) = -889.0 / 250.0;
	(*p_cst_jacobian_value)(17, 2) = -1.0;

	(*p_cst_jacobian_value)(0, 6) = cos(updataInfo->desire_yaw);
	(*p_cst_jacobian_value)(1, 6) = -sin(updataInfo->desire_yaw);
	(*p_cst_jacobian_value)(2, 6) = sin(updataInfo->desire_yaw);

	(*p_cst_jacobian_value)(8, 6) = cos(updataInfo->desire_yaw);
	(*p_cst_jacobian_value)(9, 6) = -sin(updataInfo->desire_yaw);

	(*p_cst_jacobian_value)(0, 7) = sin(updataInfo->desire_yaw);
	(*p_cst_jacobian_value)(1, 7) = cos(updataInfo->desire_yaw);
	(*p_cst_jacobian_value)(2, 7) = -cos(updataInfo->desire_yaw);

	(*p_cst_jacobian_value)(8, 7) = sin(updataInfo->desire_yaw);
	(*p_cst_jacobian_value)(9, 7) = cos(updataInfo->desire_yaw);

	(*p_cst_jacobian_value)(1, 8) = 889.0 / 250.0;
	(*p_cst_jacobian_value)(2, 8) = 889.0 / 250.0;
	(*p_cst_jacobian_value)(3, 8) = 1.0;

	(*p_cst_jacobian_value)(4, 9) = cos(updataInfo->desire_yaw);
	(*p_cst_jacobian_value)(5, 9) = -sin(updataInfo->desire_yaw);
	(*p_cst_jacobian_value)(6, 9) = sin(updataInfo->desire_yaw);

	(*p_cst_jacobian_value)(8, 9) = -cos(updataInfo->desire_yaw);
	(*p_cst_jacobian_value)(9, 9) = sin(updataInfo->desire_yaw);

	(*p_cst_jacobian_value)(4, 10) = sin(updataInfo->desire_yaw);
	(*p_cst_jacobian_value)(5, 10) = cos(updataInfo->desire_yaw);
	(*p_cst_jacobian_value)(6, 10) = -cos(updataInfo->desire_yaw);

	(*p_cst_jacobian_value)(8, 10) = -sin(updataInfo->desire_yaw);
	(*p_cst_jacobian_value)(9, 10) = -cos(updataInfo->desire_yaw);

	(*p_cst_jacobian_value)(4, 11) = 889.0 / 250.0;
	(*p_cst_jacobian_value)(5, 11) = 889.0 / 250.0;
	(*p_cst_jacobian_value)(6, 11) = 1.0;
}

void FH_constraints::GetKinematicsCstJacobianPos(Mat18x13<double> *p_cst_jacobian_pos)
{
	(*p_cst_jacobian_pos).setZero();

	(*p_cst_jacobian_pos)(0, 0) = 1.0;
	(*p_cst_jacobian_pos)(1, 0) = 1.0;
	(*p_cst_jacobian_pos)(2, 0) = 1.0;

	(*p_cst_jacobian_pos)(4, 0) = 1.0;
	(*p_cst_jacobian_pos)(5, 0) = 1.0;
	(*p_cst_jacobian_pos)(6, 0) = 1.0;

	(*p_cst_jacobian_pos)(10, 0) = 1.0;
	(*p_cst_jacobian_pos)(11, 0) = 1.0;
	(*p_cst_jacobian_pos)(12, 0) = 1.0;

	(*p_cst_jacobian_pos)(14, 0) = 1.0;
	(*p_cst_jacobian_pos)(15, 0) = 1.0;
	(*p_cst_jacobian_pos)(16, 0) = 1.0;

	(*p_cst_jacobian_pos)(0, 1) = 1.0;
	(*p_cst_jacobian_pos)(1, 1) = 1.0;
	(*p_cst_jacobian_pos)(2, 1) = 1.0;

	(*p_cst_jacobian_pos)(4, 1) = 1.0;
	(*p_cst_jacobian_pos)(5, 1) = 1.0;
	(*p_cst_jacobian_pos)(6, 1) = 1.0;

	(*p_cst_jacobian_pos)(10, 1) = 1.0;
	(*p_cst_jacobian_pos)(11, 1) = 1.0;
	(*p_cst_jacobian_pos)(12, 1) = 1.0;

	(*p_cst_jacobian_pos)(14, 1) = 1.0;
	(*p_cst_jacobian_pos)(15, 1) = 1.0;
	(*p_cst_jacobian_pos)(16, 1) = 1.0;

	(*p_cst_jacobian_pos)(1, 2) = 1.0;
	(*p_cst_jacobian_pos)(2, 2) = 1.0;
	(*p_cst_jacobian_pos)(3, 2) = 1.0;

	(*p_cst_jacobian_pos)(5, 2) = 1.0;
	(*p_cst_jacobian_pos)(6, 2) = 1.0;
	(*p_cst_jacobian_pos)(7, 2) = 1.0;

	(*p_cst_jacobian_pos)(11, 2) = 1.0;
	(*p_cst_jacobian_pos)(12, 2) = 1.0;
	(*p_cst_jacobian_pos)(13, 2) = 1.0;

	(*p_cst_jacobian_pos)(15, 2) = 1.0;
	(*p_cst_jacobian_pos)(16, 2) = 1.0;
	(*p_cst_jacobian_pos)(17, 2) = 1.0;

	(*p_cst_jacobian_pos)(0, 6) = 1.0;
	(*p_cst_jacobian_pos)(1, 6) = 1.0;
	(*p_cst_jacobian_pos)(2, 6) = 1.0;

	(*p_cst_jacobian_pos)(8, 6) = 1.0;
	(*p_cst_jacobian_pos)(9, 6) = 1.0;

	(*p_cst_jacobian_pos)(0, 7) = 1.0;
	(*p_cst_jacobian_pos)(1, 7) = 1.0;
	(*p_cst_jacobian_pos)(2, 7) = 1.0;

	(*p_cst_jacobian_pos)(8, 7) = 1.0;
	(*p_cst_jacobian_pos)(9, 7) = 1.0;

	(*p_cst_jacobian_pos)(1, 8) = 1.0;
	(*p_cst_jacobian_pos)(2, 8) = 1.0;
	(*p_cst_jacobian_pos)(3, 8) = 1.0;

	(*p_cst_jacobian_pos)(4, 9) = 1.0;
	(*p_cst_jacobian_pos)(5, 9) = 1.0;
	(*p_cst_jacobian_pos)(6, 9) = 1.0;

	(*p_cst_jacobian_pos)(8, 9) = 1.0;
	(*p_cst_jacobian_pos)(9, 9) = 1.0;

	(*p_cst_jacobian_pos)(4, 10) = 1.0;
	(*p_cst_jacobian_pos)(5, 10) = 1.0;
	(*p_cst_jacobian_pos)(6, 10) = 1.0;

	(*p_cst_jacobian_pos)(8, 10) = 1.0;
	(*p_cst_jacobian_pos)(9, 10) = 1.0;

	(*p_cst_jacobian_pos)(4, 11) = 1.0;
	(*p_cst_jacobian_pos)(5, 11) = 1.0;
	(*p_cst_jacobian_pos)(6, 11) = 1.0;
}


void FH_constraints::GetOtherCst(Vec23<double> *p_cst, Vec3<double> p_bodyPos, Vec3<double> p_bodyVel, Vec3<double> p_foothold_front, Vec3<double> p_foothold_rear)
{
	double desOmega_yaw = 0.0;
	double Ts = updataInfo->Ts;
	Vec3<double> pos = p_bodyPos;
	Vec3<double> curVel = updataInfo->curVelocity;
	Vec3<double> desVel = p_bodyVel;

	Mat3<double> rotation = Common::RotationOfZMatrix(updataInfo->desire_yaw);
	Vec3<double> distance = rotation * (p_foothold_front - p_foothold_rear);

	desOmega_yaw = (updataInfo->desire_yaw - updataInfo->curEuler(2)) / Ts;

	(*p_cst).setZero();
	(*p_cst)(0) = pos(0) - (curVel(0) + desVel(0)) * Ts / 2.0 - updataInfo->curPos(0);
	(*p_cst)(1) = pos(1) - (curVel(1) + desVel(1)) * Ts / 2.0 - updataInfo->curPos(1);

	(*p_cst)(2) = p_foothold_front(0) - updataInfo->endfoot_desire_front(0);
	(*p_cst)(3) = p_foothold_front(1) - updataInfo->endfoot_desire_front(1);
	(*p_cst)(4) = p_foothold_front(2) - updataInfo->endfoot_desire_front(2);
	(*p_cst)(5) = p_foothold_rear(0) - updataInfo->endfoot_desire_rear(0);
	(*p_cst)(6) = p_foothold_rear(1) - updataInfo->endfoot_desire_rear(1);
	(*p_cst)(7) = p_foothold_rear(2) - updataInfo->endfoot_desire_rear(2);

	(*p_cst)(8) = -(p_foothold_front(0) + p_foothold_rear(0)) / 2.0 + Ts * desVel(0) / 2.0 +
				  updataInfo->k_v(0) * (curVel(0) - desVel(0)) +
				  0.5 * sqrt(updataInfo->desire_hight / GRAVITY) * curVel(1) * desOmega_yaw + pos(0);

	(*p_cst)(9) = -(p_foothold_front(1) + p_foothold_rear(1)) / 2.0 + Ts * desVel(1) / 2.0 +
				  updataInfo->k_v(1) * (curVel(1) - desVel(1)) +
				  0.5 * sqrt(updataInfo->desire_hight / GRAVITY) * (-curVel(0) * desOmega_yaw) + pos(1);

	(*p_cst)(10) = pos(2) - (p_foothold_front(2) + p_foothold_rear(2)) / 2.0 - updataInfo->desire_hight;


	(*p_cst)(11) = pow(p_foothold_front(0) - updataInfo->terrain_now->split[0] , 2);
	(*p_cst)(12) = pow(p_foothold_front(0) - updataInfo->terrain_now->split[1] , 2);
	(*p_cst)(13) = pow(p_foothold_front(0) - updataInfo->terrain_now->split[2] , 2);
	(*p_cst)(14) = pow(p_foothold_front(0) - updataInfo->terrain_now->split[3] , 2);
	(*p_cst)(15) = pow(p_foothold_front(0) - updataInfo->terrain_now->split[4] , 2);
	(*p_cst)(16) = pow(p_foothold_front(0) - updataInfo->terrain_now->split[5] , 2);

	(*p_cst)(17) = pow(p_foothold_rear(0) - updataInfo->terrain_now->split[0] , 2);
	(*p_cst)(18) = pow(p_foothold_rear(0) - updataInfo->terrain_now->split[1] , 2);
	(*p_cst)(19) = pow(p_foothold_rear(0) - updataInfo->terrain_now->split[2] , 2);
	(*p_cst)(20) = pow(p_foothold_rear(0) - updataInfo->terrain_now->split[3] , 2);
	(*p_cst)(21) = pow(p_foothold_rear(0) - updataInfo->terrain_now->split[4] , 2);
	(*p_cst)(22) = pow(p_foothold_rear(0) - updataInfo->terrain_now->split[5] , 2);
}


void FH_constraints::GetOtherCstBound(double *ub, double *lb)
{
	ub[0] = 0.0;
	ub[1] = 0.0;
	ub[2] = 0.1;
	ub[3] = 0.03;
	ub[4] = 0.0;
	ub[5] = 0.1;
	ub[6] = 0.03;
	ub[7] = 0.0;
	ub[8] = 0.0;
	ub[9] = 0.0;
	ub[10] = 0.1;

	ub[11] = LIMIT_INFINITE;
	ub[12] = LIMIT_INFINITE;
	ub[13] = LIMIT_INFINITE;
	ub[14] = LIMIT_INFINITE;
	ub[15] = LIMIT_INFINITE;
	ub[16] = LIMIT_INFINITE;
	ub[17] = LIMIT_INFINITE;
	ub[18] = LIMIT_INFINITE;
	ub[19] = LIMIT_INFINITE;
	ub[20] = LIMIT_INFINITE;
	ub[21] = LIMIT_INFINITE;
	ub[22] = LIMIT_INFINITE;



	lb[0] = -0.0;
	lb[1] = 0.0;
	lb[2] = -0.1;
	lb[3] = -0.03;
	lb[4] = -0.0;
	lb[5] = -0.1;
	lb[6] = -0.03;
	lb[7] = -0.0;
	lb[8] = -0.0;
	lb[9] = 0.0;
	lb[10] = -0.1;

	lb[11] = pow(updataInfo->terrain_now->safeArea[0].first , 2);
	lb[12] = pow(updataInfo->terrain_now->safeArea[1].first , 2);
	lb[13] = pow(updataInfo->terrain_now->safeArea[2].first , 2);
	lb[14] = pow(updataInfo->terrain_now->safeArea[3].first , 2);
	lb[15] = pow(updataInfo->terrain_now->safeArea[4].first , 2);
	lb[16] = pow(updataInfo->terrain_now->safeArea[5].first , 2);
	lb[17] = pow(updataInfo->terrain_now->safeArea[0].first , 2);
	lb[18] = pow(updataInfo->terrain_now->safeArea[1].first , 2);
	lb[19] = pow(updataInfo->terrain_now->safeArea[2].first , 2);
	lb[20] = pow(updataInfo->terrain_now->safeArea[3].first , 2);
	lb[21] = pow(updataInfo->terrain_now->safeArea[4].first , 2);
	lb[22] = pow(updataInfo->terrain_now->safeArea[5].first , 2);
}

void FH_constraints::GetOtherCstJacobian(Mat23x12<double> *p_cst_jacobian_value,Vec3<double> p_bodyPos,Vec3<double> p_bodyVel,Vec3<double> p_foothold_front,Vec3<double> p_foothold_rear)
{
	double Ts = updataInfo->Ts;

	(*p_cst_jacobian_value).setZero();

	(*p_cst_jacobian_value)(0, 0) = 1.0;
	(*p_cst_jacobian_value)(0, 3) = -Ts / 2.0;

	(*p_cst_jacobian_value)(1, 1) = 1.0;
	(*p_cst_jacobian_value)(1, 4) = -Ts / 2.0;

	(*p_cst_jacobian_value)(2, 6) = 1.0;
	(*p_cst_jacobian_value)(3, 7) = 1.0;
	(*p_cst_jacobian_value)(4, 8) = 1.0;
	(*p_cst_jacobian_value)(5, 9) = 1.0;
	(*p_cst_jacobian_value)(6, 10) = 1.0;
	(*p_cst_jacobian_value)(7, 11) = 1.0;

	(*p_cst_jacobian_value)(8, 0) = 1.0;
	(*p_cst_jacobian_value)(8, 3) = Ts / 2.0 - updataInfo->k_v(0);
	(*p_cst_jacobian_value)(8, 6) = -0.5;
	(*p_cst_jacobian_value)(8, 9) = -0.5;

	(*p_cst_jacobian_value)(9, 1) = 1.0;
	(*p_cst_jacobian_value)(9, 4) = Ts / 2.0 - updataInfo->k_v(1);
	(*p_cst_jacobian_value)(9, 7) = -0.5;
	(*p_cst_jacobian_value)(9, 10) = -0.5;

	(*p_cst_jacobian_value)(10, 2) = 1.0;
	(*p_cst_jacobian_value)(10, 8) = -0.5;
	(*p_cst_jacobian_value)(10, 11) = -0.5;

	(*p_cst_jacobian_value)(11,6) = 2.0 * p_foothold_front(0) - 2.0 * updataInfo->terrain_now->split[0];
	(*p_cst_jacobian_value)(12,6) = 2.0 * p_foothold_front(0) - 2.0 * updataInfo->terrain_now->split[1];
	(*p_cst_jacobian_value)(13,6) = 2.0 * p_foothold_front(0) - 2.0 * updataInfo->terrain_now->split[2];
	(*p_cst_jacobian_value)(14,6) = 2.0 * p_foothold_front(0) - 2.0 * updataInfo->terrain_now->split[3];
	(*p_cst_jacobian_value)(15,6) = 2.0 * p_foothold_front(0) - 2.0 * updataInfo->terrain_now->split[4];
	(*p_cst_jacobian_value)(16,6) = 2.0 * p_foothold_front(0) - 2.0 * updataInfo->terrain_now->split[5];

	(*p_cst_jacobian_value)(17,9) = 2.0 * p_foothold_rear(0) - 2.0 * updataInfo->terrain_now->split[0];
	(*p_cst_jacobian_value)(18,9) = 2.0 * p_foothold_rear(0) - 2.0 * updataInfo->terrain_now->split[1];
	(*p_cst_jacobian_value)(19,9) = 2.0 * p_foothold_rear(0) - 2.0 * updataInfo->terrain_now->split[2];
	(*p_cst_jacobian_value)(20,9) = 2.0 * p_foothold_rear(0) - 2.0 * updataInfo->terrain_now->split[3];
	(*p_cst_jacobian_value)(21,9) = 2.0 * p_foothold_rear(0) - 2.0 * updataInfo->terrain_now->split[4];
	(*p_cst_jacobian_value)(22,9) = 2.0 * p_foothold_rear(0) - 2.0 * updataInfo->terrain_now->split[5];

}

void FH_constraints::GetOtherCstJacobianPos(Mat23x12<double> *p_cst_jacobian_pos)
{

	(*p_cst_jacobian_pos).setZero();

	(*p_cst_jacobian_pos)(0, 0) = 1.0;
	(*p_cst_jacobian_pos)(0, 3) = 1.0;

	(*p_cst_jacobian_pos)(1, 1) = 1.0;
	(*p_cst_jacobian_pos)(1, 4) = 1.0;

	(*p_cst_jacobian_pos)(2, 6) = 1.0;
	(*p_cst_jacobian_pos)(3, 7) = 1.0;
	(*p_cst_jacobian_pos)(4, 8) = 1.0;
	(*p_cst_jacobian_pos)(5, 9) = 1.0;
	(*p_cst_jacobian_pos)(6, 10) = 1.0;
	(*p_cst_jacobian_pos)(7, 11) = 1.0;

	(*p_cst_jacobian_pos)(8, 0) = 1.0;
	(*p_cst_jacobian_pos)(8, 3) = 1.0;
	(*p_cst_jacobian_pos)(8, 6) = 1.0;
	(*p_cst_jacobian_pos)(8, 9) = 1.0;

	(*p_cst_jacobian_pos)(9, 1) = 1.0;
	(*p_cst_jacobian_pos)(9, 4) = 1.0;
	(*p_cst_jacobian_pos)(9, 7) = 1.0;
	(*p_cst_jacobian_pos)(9, 10) = 1.0;

	(*p_cst_jacobian_pos)(10, 2) = 1.0;
	(*p_cst_jacobian_pos)(10, 8) = 1.0;
	(*p_cst_jacobian_pos)(10, 11) = 1.0;

	(*p_cst_jacobian_pos)(11,6) = 1.0;
	(*p_cst_jacobian_pos)(12,6) = 1.0;
	(*p_cst_jacobian_pos)(13,6) = 1.0;
	(*p_cst_jacobian_pos)(14,6) = 1.0;
	(*p_cst_jacobian_pos)(15,6) = 1.0;
	(*p_cst_jacobian_pos)(16,6) = 1.0;

	(*p_cst_jacobian_pos)(17,9) = 1.0;
	(*p_cst_jacobian_pos)(18,9) = 1.0;
	(*p_cst_jacobian_pos)(19,9) = 1.0;
	(*p_cst_jacobian_pos)(20,9) = 1.0;
	(*p_cst_jacobian_pos)(21,9) = 1.0;
	(*p_cst_jacobian_pos)(22,9) = 1.0;

}

void FH_constraints::GetOtherCstHessian(double* lambda, Mat12<double>* p_cst_hessian_value)
{
	(*p_cst_hessian_value).setZero();

	(*p_cst_hessian_value)(6,6) = lambda[11] * 2.0;
	(*p_cst_hessian_value)(6,6) += lambda[12] * 2.0;
	(*p_cst_hessian_value)(6,6) += lambda[13] * 2.0;
	(*p_cst_hessian_value)(6,6) += lambda[14] * 2.0;
	(*p_cst_hessian_value)(6,6) += lambda[15] * 2.0;
	(*p_cst_hessian_value)(6,6) += lambda[16] * 2.0;

	(*p_cst_hessian_value)(9,9) += lambda[17] * 2.0;
	(*p_cst_hessian_value)(9,9) += lambda[18] * 2.0;
	(*p_cst_hessian_value)(9,9) += lambda[19] * 2.0;
	(*p_cst_hessian_value)(9,9) += lambda[20] * 2.0;
	(*p_cst_hessian_value)(9,9) += lambda[21] * 2.0;
	(*p_cst_hessian_value)(9,9) += lambda[22] * 2.0;
}

void FH_constraints::GetOtherCstHessianPos(Mat12<double>* p_cst_hessian_pos)
{
	(*p_cst_hessian_pos).setZero();

	(*p_cst_hessian_pos)(6,6) = 1.0;
	(*p_cst_hessian_pos)(6,6) += 1.0;
	(*p_cst_hessian_pos)(6,6) += 1.0;
	(*p_cst_hessian_pos)(6,6) += 1.0;
	(*p_cst_hessian_pos)(6,6) += 1.0;
	(*p_cst_hessian_pos)(6,6) += 1.0;

	(*p_cst_hessian_pos)(9,9) += 1.0;
	(*p_cst_hessian_pos)(9,9) += 1.0;
	(*p_cst_hessian_pos)(9,9) += 1.0;
	(*p_cst_hessian_pos)(9,9) += 1.0;
	(*p_cst_hessian_pos)(9,9) += 1.0;
	(*p_cst_hessian_pos)(9,9) += 1.0;
}