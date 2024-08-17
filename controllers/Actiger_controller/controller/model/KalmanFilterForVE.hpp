/**************************************************************************
Copyright:Action
Author: 
Date:2021-03-27
Description:Kalman Filter of Velocity Estimate of AcTiger
**************************************************************************/

#ifndef KALMANFILTERFORVE_HPP
#define KALMANFILTERFORVE_HPP

#include "cppTypes.h"
#include <vector>


class KalmanFilterForVE
{
public:
	KalmanFilterForVE(double mass_input, double period_input);
    virtual ~KalmanFilterForVE();
	void CalStddevXdot(std::vector<bool> contactlist_input,
		int contact_num_input,
		Vec3<double> b_xdot_lf_input,
		Vec3<double> b_xdot_lr_input,
		Vec3<double> b_xdot_rf_input,
		Vec3<double> b_xdot_rr_input);
	void CalFzAverage(std::vector<bool> contactlist_input, int contact_num_input);
	void MakeMatrixF();
	void MakeVectorU();
	void StatePredict();
	void CovPredict();
	void GetMeasurementError();
	void GetK();
	void StateUpdate();
	void CovUpdate();

	//*************************************************************
	void Init(Vec3<double> v_k_1_input,
			  DVec<double> f_k_input,
			  DVec<double> f_k_1_input,
			  Vec3<double> xdot_input,
			  Vec3<double> p_k_input,
			  Vec3<double> p_k_1_input,
			  Vec3<double> a_k_1_input);
	void Ready(std::vector<bool> contactlist_input,
			   int contact_num_input,
			   Vec3<double> b_xdot_lf_input,
			   Vec3<double> b_xdot_lr_input,
			   Vec3<double> b_xdot_rf_input,
			   Vec3<double> b_xdot_rr_input);
	void Filter();
	Vec6<double> Result();
	//*************************************************************
	
	//coefficients used by prediction of Cov
	double alpha_1;
	double alpha_2;
	//standard deviation of the velocity component contributed by each stance foot
	double std_velocity_contribution_x;
	double std_velocity_contribution_y;
	double std_velocity_contribution_z;
	//process noise of measurement of position
	double Q_pos_x;
	double Q_pos_y;
	double Q_pos_z;
	//robot's mass
	double mass;
	//sampling period
	double period;

	//state transition matrix F
	Mat6<double> matrix_F = Mat6<double>::Zero(6, 6);
	//control vector U
	Vec6<double> vector_u = Vec6<double>::Zero(6);
	//process noise matrix Q
	Mat6<double> matrix_Q = Mat6<double>::Zero(6, 6);
	//Cov matrix P after updated
	Mat6<double> matrix_P = Mat6<double>::Zero(6, 6);
	//Cov matrix P after predicted
	Mat6<double> matrix_P_predict = Mat6<double>::Zero(6, 6);
	//measurement matrix H
	Mat3x6<double> matrix_H = Mat3x6<double>::Zero(3, 6);
	//error vector y between measurement and prediction
	Vec3<double> vector_y = Vec3<double>::Zero(3);
	//measurement noise of IMU
	Mat3<double> matrix_R = Mat3<double>::Zero(3, 3);
	//Kalman gain matrix K
	Mat6x3<double> matrix_K = Mat6x3<double>::Zero(6, 3);
	//last velocity
	Vec3<double> v_k_1 = Vec3<double>::Zero(3);
	//current footend force
	DVec<double> f_k = DVec<double>::Zero(12);
	//last footend force
	DVec<double> f_k_1 = DVec<double>::Zero(12);
	//last position
	Vec3<double> p_k_1 = Vec3<double>::Zero(3);
	//current position
	Vec3<double> p_k = Vec3<double>::Zero(3);
	//last acceleration
	Vec3<double> a_k_1 = Vec3<double>::Zero(3);
	//state of robot
	Vec6<double> x = Vec6<double>::Zero(6);
	//current velocity without filter
	Vec3<double> xdot = Vec3<double>::Zero(3);
	//predicted state of robot
	Vec6<double> x_predict = Vec6<double>::Zero(6);
	//average of the footend force in Z direction between samples
	double f_z_average;
};


#endif //KALMANFILTERFORVE_HPP


