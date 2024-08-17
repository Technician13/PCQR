/**
 * @file parameters.cpp
 * @brief Actiger参数类Parameters
 * @author Bin & Tech13
 * @version 1.1
 * @date 2021-08-19
 * 
 * @copyright  Copyright  (c)  2021  Action
 * 
 */

#include "parameters.hpp"
#include <iostream>

using namespace quadruped;

pthread_mutex_t model_mutex; /**< 模型读写互斥锁 */
pthread_mutex_t f_des_mutex; /**< 模型预测控制器输出读写互斥锁 */
pthread_mutex_t ref_mutex;	 /**< 规划读写互斥锁 */
handle_t handle = {0};		 /**< 标志位结构体 */

/**
 * @brief Parameters类的构造函数，通过调用成员函数完成数据成员的初值装载
 */
Parameters::Parameters(void)
{
	SetModelPara();
	SetMotionPlanPara();
	SetLegControlPara();

	std::cout << "Parameters Init" << std::endl;
}

/**
 * @brief Parameters类的析构函数
 */
Parameters::~Parameters(void) {}

/**
 * @brief Actiger模型参数初值装载
 */
void Parameters::SetModelPara(void)
{
	lf_hip_pos_body << 0.33, 0.07, 0.0;
	lr_hip_pos_body << -0.33, 0.055, 0.0;
	rf_hip_pos_body << 0.33, -0.07, 0.0;
	rr_hip_pos_body << -0.33, -0.055, 0.0;
}

/**
 * @brief Actiger规划器参数初值装载
 */
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

/**
 * @brief Actiger单腿控制器参数初值装载
 */
void Parameters::SetLegControlPara(void)
{
	leg_kpJoint << 300.0, 300.0, 300.0;
	leg_kdJoint << 25.0, 25.0, 25.0;
}