#ifndef PARAMETERS_HPP
#define PARAMETERS_HPP

#include "cppTypes.h"
#include "commonFunction.hpp"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <strings.h>
#include <pthread.h>

extern pthread_mutex_t model_mutex;	
extern pthread_mutex_t f_des_mutex;	
extern pthread_mutex_t ref_mutex;
extern handle_t handle;

namespace quadruped
{
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

		Vec3<double> lf_hip_pos_body;		
		Vec3<double> lr_hip_pos_body;
		Vec3<double> rf_hip_pos_body;
		Vec3<double> rr_hip_pos_body;

		Vec3<double> wbc_kp_pos;
		Vec3<double> wbc_kd_pos;
		Vec3<double> wbc_kp_kin_pos;
		Vec3<double> wbc_kp_ori;
		Vec3<double> wbc_kd_ori;
		Vec3<double> wbc_kp_kin_ori;
		Vec3<double> wbc_kp_link;
		Vec3<double> wbc_kd_link;
		Vec3<double> wbc_kp_kin_link;
		Mat6<double> wbc_w_floating;
		Mat12<double> wbc_w_rf;

		double Ts;
		Vec2<double> plan_k_v;

		Vec2<double> plan_foothold_Q;
		Vec2<double> plan_foothold_R;
		Vec2<double> plan_endfoot_max;
		Vec6<double> plan_spatial_max;
		Vec6<double> plan_spatial_dot_max;
		Vec6<double> plan_spatial_min;
		Vec6<double> plan_spatial_dot_min;
		Vec3<double> plan_force_max;
		Vec3<double> plan_force_min;

		Vec3<double> leg_kpJoint;
		Vec3<double> leg_kdJoint;	
	};
}

#endif // !PARAMETERS_HPP