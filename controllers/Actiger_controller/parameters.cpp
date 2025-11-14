#include "parameters.hpp"
#include <iostream>

using namespace quadruped;

pthread_mutex_t model_mutex; 
pthread_mutex_t f_des_mutex; 
pthread_mutex_t ref_mutex;	 
handle_t handle = {0};		 


Parameters::Parameters(void)
{
	SetModelPara();
	SetMotionPlanPara();
	SetLegControlPara();

	std::cout << "Parameters Init" << std::endl;
}


Parameters::~Parameters(void) {}


void Parameters::SetModelPara(void)
{
	lf_hip_pos_body << 0.33, 0.07, 0.0;
	lr_hip_pos_body << -0.33, 0.055, 0.0;
	rf_hip_pos_body << 0.33, -0.07, 0.0;
	rr_hip_pos_body << -0.33, -0.055, 0.0;
}


void Parameters::SetMotionPlanPara(void)
{
	Ts = 0.3;
	plan_k_v << 0.10, 0.10;
	plan_endfoot_max << 0.3, 0.15;
	plan_foothold_Q << 10.0, 10.0;
	plan_foothold_R << 1.0, 1.0;
	plan_spatial_max << PI / 60.0, PI / 60.0, 10000.0 * PI, 100000.0, 100000.0, 100000.0;
	plan_spatial_dot_max << PI, PI, PI, 2.0, 1.5, 6.0;
	plan_spatial_min << -PI / 60.0, -PI / 60.0, -10000.0 * PI, -100000.0, -100000.0, -100000.0;
	plan_spatial_dot_min << -PI, -PI, -PI, -2.0, -1.5, -6.0;
	plan_force_max << 300.0, 300.0, 300.0;
	plan_force_min << -300.0, -300.0, 20.0;
}

void Parameters::SetLegControlPara(void)
{
	leg_kpJoint << 300.0, 300.0, 300.0;
	leg_kdJoint << 25.0, 25.0, 25.0;
}