/**
 * @file parameters.hpp
 * @brief Actiger参数类Parameters头文件
 * @author Bin & Tech13
 * @version 1.1
 * @date 2021-08-19
 * 
 * @copyright  Copyright  (c)  2021  Action
 * 
 */

#ifndef PARAMETERS_HPP
#define PARAMETERS_HPP

#include "cppTypes.h"
#include "commonFunction.hpp"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <strings.h>
#include <pthread.h>

extern pthread_mutex_t model_mutex;		/**< 模型读写互斥锁 */
extern pthread_mutex_t f_des_mutex;		/**< 模型预测控制器输出读写互斥锁 */
extern pthread_mutex_t ref_mutex;		/**< 规划读写互斥锁 */
extern handle_t handle;					/**< 标志位结构体 */

namespace quadruped
{
	/**
	 * @brief Actiger参数类
	 */
	class Parameters
	{
	private:
		
	protected:

	public:
		explicit Parameters(void);
		virtual ~Parameters(void);

		void SetModelPara(void);
		void SetMotionPlanPara(void);
		void SetLegControlPara(void);

		/*********************************************************Model参数*********************************************************/
		/** @brief Actiger左前髋关节在机体坐标系下的位置 */
		Vec3<double> lf_hip_pos_body;		
		/** @brief Actiger左后髋关节在机体坐标系下的位置 */
		Vec3<double> lr_hip_pos_body;
		/** @brief Actiger右前髋关节在机体坐标系下的位置 */
		Vec3<double> rf_hip_pos_body;
		/** @brief Actiger右后髋关节在机体坐标系下的位置 */
		Vec3<double> rr_hip_pos_body;

		/**********************************************************WBC参数**********************************************************/
		/** @brief WBC预留接口 */
		Vec3<double> wbc_kp_pos;
		/** @brief WBC预留接口 */
		Vec3<double> wbc_kd_pos;
		/** @brief WBC预留接口 */
		Vec3<double> wbc_kp_kin_pos;
		/** @brief WBC预留接口 */
		Vec3<double> wbc_kp_ori;
		/** @brief WBC预留接口 */
		Vec3<double> wbc_kd_ori;
		/** @brief WBC预留接口 */
		Vec3<double> wbc_kp_kin_ori;
		/** @brief WBC预留接口 */
		Vec3<double> wbc_kp_link;
		/** @brief WBC预留接口 */
		Vec3<double> wbc_kd_link;
		/** @brief WBC预留接口 */
		Vec3<double> wbc_kp_kin_link;
		/** @brief WBC预留接口 */
		Mat6<double> wbc_w_floating;
		/** @brief WBC预留接口 */
		Mat12<double> wbc_w_rf;

		/*******************************************************MotionPlan参数******************************************************/
		/** @brief 一个完整步态周期中的支撑相所占时间 */
		double Ts;
		/** @brief 落足点规划时的反馈增益参数 */
		Vec2<double> plan_k_v;


		/** @brief MotionPlan预留接口 */
		Vec2<double> plan_foothold_Q;
		/** @brief MotionPlan预留接口 */
		Vec2<double> plan_foothold_R;
		/** @brief MotionPlan预留接口 */
		Vec2<double> plan_endfoot_max;
		/** @brief MotionPlan预留接口 */
		Vec6<double> plan_spatial_max;
		/** @brief MotionPlan预留接口 */
		Vec6<double> plan_spatial_dot_max;
		/** @brief MotionPlan预留接口 */
		Vec6<double> plan_spatial_min;
		/** @brief MotionPlan预留接口 */
		Vec6<double> plan_spatial_dot_min;
		/** @brief MotionPlan预留接口 */
		Vec3<double> plan_force_max;
		/** @brief MotionPlan预留接口 */
		Vec3<double> plan_force_min;

		/*******************************************************LegControl参数*******************************************************/
		/** @brief 摆动腿控制的比例反馈增益 */
		Vec3<double> leg_kpJoint;
		/** @brief 摆动腿控制的微分反馈增益 */
		Vec3<double> leg_kdJoint;	
	};
}

#endif // !PARAMETERS_HPP