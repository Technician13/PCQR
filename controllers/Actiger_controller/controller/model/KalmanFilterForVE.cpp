/**************************************************************************
Copyright:Action
Author: 
Date:2021-03-27
Description:Kalman Filter of Velocity Estimate of AcTiger
**************************************************************************/

#include "KalmanFilterForVE.hpp"
#include <math.h> 
#include <iostream>

/**************************************************************************
*Summary: constructor
*Function: initialization of variables
*Parameters:
*     mass_input : mass of robot   
*     period_input : sampling time
*Return : None
**************************************************************************/

KalmanFilterForVE::KalmanFilterForVE(double mass_input,double period_input):mass(mass_input),period(period_input)
{
	matrix_P << 1.0,   0.0,   0.0,   0.0,   0.0,   0.0,
				0.0,   1.0,   0.0,   0.0,   0.0,   0.0,
				0.0,   0.0,   1.0,   0.0,   0.0,   0.0,
				0.0,   0.0,   0.0,   100.0, 0.0,   0.0,
				0.0,   0.0,   0.0,   0.0,   100.0, 0.0,
				0.0,   0.0,   0.0,   0.0,   0.0,   100.0;

	matrix_H << 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
				0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
				0.0, 0.0, 0.0, 0.0, 0.0, 1.0;

	matrix_R << 0.1, 0.0, 0.0,
				0.0, 0.1, 0.0,
				0.0, 0.0, 0.1;

	alpha_1 = 0.5;
	alpha_2 = 0.5;

	Q_pos_x = 1.0;
	Q_pos_y = 1.0;
	Q_pos_z = 1.0;	
	
}

/**************************************************************************
*Summary: destructor
*Function: None
*Parameters: None
*Return : None
**************************************************************************/

KalmanFilterForVE::~KalmanFilterForVE()
{

}

/**************************************************************************
*Summary: CalStddevXdot
*Function: calculate standard deviation of the 
		   velocity component contributed by each stance foot
*Parameters: 
*	  contactlist_input : footend contact list
*	  contact_num_input : number of stance foot
*	  b_xdot_lf_input   : lf stance footend velocity under base frame
*	  b_xdot_lr_input   : lr stance footend velocity under base frame
*	  b_xdot_rf_input   : rf stance footend velocity under base frame
*	  b_xdot_rr_input   : rr stance footend velocity under base frame
*Return : None
**************************************************************************/

void KalmanFilterForVE::CalStddevXdot(std::vector<bool> contactlist_input,
	int contact_num_input, 
	Vec3<double> b_xdot_lf_input,
	Vec3<double> b_xdot_lr_input,
	Vec3<double> b_xdot_rf_input,
	Vec3<double> b_xdot_rr_input)
{
	Vec3<double> average_b_xdot = Vec3<double>::Zero(3);
	average_b_xdot = (b_xdot_lf_input + b_xdot_lr_input + b_xdot_rf_input + b_xdot_rr_input) / (double)(contact_num_input);
	double average_b_xdot_x = average_b_xdot[0];
	double average_b_xdot_y = average_b_xdot[1];
	double average_b_xdot_z = average_b_xdot[2];

	double cov_sum_x = (double)contactlist_input[0] * (double)pow((b_xdot_lf_input[0] - average_b_xdot_x), 2) + \
		(double)contactlist_input[1] * (double)pow((b_xdot_lr_input[0] - average_b_xdot_x), 2) + \
		(double)contactlist_input[2] * (double)pow((b_xdot_rf_input[0] - average_b_xdot_x), 2) + \
		(double)contactlist_input[3] * (double)pow((b_xdot_rr_input[0] - average_b_xdot_x), 2);
	double cov_sum_y = (double)contactlist_input[0] * (double)pow((b_xdot_lf_input[1] - average_b_xdot_y), 2) + \
		(double)contactlist_input[1] * (double)pow((b_xdot_lr_input[1] - average_b_xdot_y), 2) + \
		(double)contactlist_input[2] * (double)pow((b_xdot_rf_input[1] - average_b_xdot_y), 2) + \
		(double)contactlist_input[3] * (double)pow((b_xdot_rr_input[1] - average_b_xdot_y), 2);
	double cov_sum_z = (double)contactlist_input[0] * (double)pow((b_xdot_lf_input[2] - average_b_xdot_z), 2) + \
		(double)contactlist_input[1] * (double)pow((b_xdot_lr_input[2] - average_b_xdot_z), 2) + \
		(double)contactlist_input[2] * (double)pow((b_xdot_rf_input[2] - average_b_xdot_z), 2) + \
		(double)contactlist_input[3] * (double)pow((b_xdot_rr_input[2] - average_b_xdot_z), 2);

	double cov_b_xdot_x = cov_sum_x / (double)(contact_num_input);
	double cov_b_xdot_y = cov_sum_y / (double)(contact_num_input);
	double cov_b_xdot_z = cov_sum_z / (double)(contact_num_input);

	std_velocity_contribution_x = (double)sqrt(cov_b_xdot_x);
	std_velocity_contribution_y = (double)sqrt(cov_b_xdot_y);
	std_velocity_contribution_z = (double)sqrt(cov_b_xdot_z);
}

/**************************************************************************
*Summary: CalFzAverage
*Function: calculate average of the footend force 
		   in Z direction between samples
*Parameters: 
*	  contactlist_input : footend contact list
*	  contact_num_input : number of stance foot
*Return : None
**************************************************************************/

void KalmanFilterForVE::CalFzAverage(std::vector<bool> contactlist_input, int contact_num_input)
{
	f_z_average = (double)(contactlist_input[0]) * (f_k[2] - f_k_1[2]) + \
		(double)(contactlist_input[1]) * (f_k[5] - f_k_1[5]) + \
		(double)(contactlist_input[2]) * (f_k[8] - f_k_1[8]) + \
		(double)(contactlist_input[3]) * (f_k[11] - f_k_1[11]);
	f_z_average = f_z_average / (double)(contact_num_input);
	
}

/**************************************************************************
*Summary: MakeMatrixF
*Function: make state transition matrix F
*Parameters: None
*Return : None
**************************************************************************/

void KalmanFilterForVE::MakeMatrixF()
{
		matrix_F(0, 0) = (double)1.0;
		matrix_F(1, 1) = (double)1.0;
		matrix_F(2, 2) = (double)1.0;

		matrix_F(0, 3) = (double)period;
		matrix_F(1, 4) = (double)period;
		matrix_F(2, 5) = (double)period;

		matrix_F(3, 3) = (double)1.0;
		matrix_F(4, 4) = (double)1.0;
		matrix_F(5, 5) = (double)1.0;
}

/**************************************************************************
*Summary: MakeMatrixF
*Function: make control vector U
*Parameters: None
*Return : None
**************************************************************************/

void KalmanFilterForVE::MakeVectorU()
{	
	vector_u(0) = (double)0.5 * a_k_1(0) * period * period;
	vector_u(1) = (double)0.5 * a_k_1(2) * period * period;
	vector_u(2) = (double)0.5 * a_k_1(1) * period * period;

	vector_u(3) = a_k_1(0) * period;
	vector_u(4) = a_k_1(2) * period;
	vector_u(5) = a_k_1(1) * period;
}

/**************************************************************************
*Summary: StatePredict
*Function: state transition function
*Parameters: None
*Return : None
**************************************************************************/

void KalmanFilterForVE::StatePredict()
{	
	x.block(0, 0, 3, 1) = p_k_1;
	x.block(3, 0, 3, 1) = v_k_1;
	
	x_predict = matrix_F * x + vector_u;
}

/**************************************************************************
*Summary: CovPredict
*Function: prediction of Cov
*Parameters: None
*Return : None
**************************************************************************/

void KalmanFilterForVE::CovPredict()
{
	matrix_Q(0, 0) = Q_pos_x;
	matrix_Q(1, 1) = Q_pos_y;
	matrix_Q(2, 2) = Q_pos_z;
	matrix_Q(3, 3) = pow((alpha_1*std_velocity_contribution_x) + alpha_2*(1 - alpha_1)*abs(f_z_average), 2);
	matrix_Q(4, 4) = pow((alpha_1*std_velocity_contribution_y) + alpha_2*(1 - alpha_1)*abs(f_z_average), 2);
	matrix_Q(5, 5) = pow((alpha_1*std_velocity_contribution_z) + alpha_2*(1 - alpha_1)*abs(f_z_average), 2);
	

	matrix_P_predict = matrix_F * matrix_P * matrix_F.transpose() + matrix_Q;
}

/**************************************************************************
*Summary: GetMeasurementError
*Function: calculate error between measurement and prediction
*Parameters: None
*Return : None
**************************************************************************/

void KalmanFilterForVE::GetMeasurementError()
{
	Vec3<double> temp = Vec3<double>::Zero(3);
	temp = xdot;

	vector_y = temp - matrix_H * x_predict;
}

/**************************************************************************
*Summary: GetK
*Function: calculate Kalman gain matrix K
*Parameters: None
*Return : None
**************************************************************************/

void KalmanFilterForVE::GetK()
{
	matrix_K = matrix_P_predict * matrix_H.transpose() * (matrix_H * matrix_P_predict * matrix_H.transpose() + matrix_R).inverse();
}

/**************************************************************************
*Summary: StateUpdate
*Function: update state
*Parameters: None
*Return : None
**************************************************************************/

void KalmanFilterForVE::StateUpdate()
{
	x = x_predict + matrix_K * vector_y;
}

/**************************************************************************
*Summary: Covupdate
*Function: update Cov matrix
*Parameters: None
*Return : None
**************************************************************************/

void KalmanFilterForVE::CovUpdate()
{
	matrix_P = (Mat6<double>::Identity(6, 6) - matrix_K * matrix_H) * matrix_P_predict;
}

/**************************************************************************
*Summary: Init
*Function: init Kalman Filter for Velocity Estimate
*Parameters: 
*	  v_k_1_input : last velocity
*	  f_k_input   : current footend force
*	  f_k_1_input : last footend force
*	  xdot_input  : current velocity without filter
*	  p_k_input   : current position
*	  p_k_1_input : last position
*     a_k_1_input : last acceleration
*Return : None
**************************************************************************/

void KalmanFilterForVE::Init(Vec3<double> v_k_1_input,
						DVec<double> f_k_input,
						DVec<double> f_k_1_input,
						Vec3<double> xdot_input,
						Vec3<double> p_k_input,
						Vec3<double> p_k_1_input,
	                    Vec3<double> a_k_1_input)
{
	v_k_1 = v_k_1_input;
	f_k = f_k_input;
	f_k_1 = f_k_1_input; 
	xdot = xdot_input;
	p_k = p_k_input; 
	p_k_1 = p_k_1_input;
	a_k_1 = a_k_1_input;	
}

/**************************************************************************
*Summary: Ready
*Function: get some necessary components for Kalman Filter
*Parameters: 
*	  contactlist_input : footend contact list
*	  contact_num_input : number of stance foot
*	  b_xdot_lf_input   : lf stance footend velocity under base frame
*	  b_xdot_lr_input   : lr stance footend velocity under base frame
*	  b_xdot_rf_input   : rf stance footend velocity under base frame
*	  b_xdot_rr_input   : rr stance footend velocity under base frame
*Return : None
**************************************************************************/

void KalmanFilterForVE::Ready(std::vector<bool> contactlist_input,
						 int contact_num_input,
						 Vec3<double> b_xdot_lf_input,
						 Vec3<double> b_xdot_lr_input,
						 Vec3<double> b_xdot_rf_input,
						 Vec3<double> b_xdot_rr_input)
{
	CalStddevXdot(contactlist_input,
				  contact_num_input,
				  b_xdot_lf_input,
				  b_xdot_lr_input,
				  b_xdot_rf_input,
				  b_xdot_rr_input);
	CalFzAverage(contactlist_input, contact_num_input);
	MakeMatrixF();
	MakeVectorU();
}

/**************************************************************************
*Summary: Filter
*Function: run Kalman Filter
*Parameters: None
*Return : None
**************************************************************************/

void KalmanFilterForVE::Filter()
{
	StatePredict();
	CovPredict();
	GetMeasurementError();
	GetK();
	StateUpdate();
	CovUpdate();
}

/**************************************************************************
*Summary: Result
*Function: get Kalman Filter's result
*Parameters: None
*Return : 
*     x : Kalman Filter's result
**************************************************************************/

Vec6<double> KalmanFilterForVE::Result()
{
	return x;
}
