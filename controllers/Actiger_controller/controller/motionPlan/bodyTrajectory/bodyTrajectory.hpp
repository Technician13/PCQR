#ifndef BODY_TRA_HPP
#define BODY_TRA_HPP


#include "parameters.hpp"


namespace quadruped
{
	
	class BodyTrajectory
	{
	public:

		double Ts;

		int grid;

		Vec3<double> normal_vec;

		bool is_lf_rr_stand;
		Vec3<double> lf_hip_pos_body;
		Vec3<double> lr_hip_pos_body;
		Vec3<double> rf_hip_pos_body;
		Vec3<double> rr_hip_pos_body;

		DMat<double> return_spatial_list; 
		DMat<double> return_spatial_dot_list; 

		explicit BodyTrajectory(Parameters* p_para);
		virtual ~BodyTrajectory();

		void GetBodyTrajectory(void);
		void UpdateInfo(bool p_is_lf_rr_stand, double p_Ts,
						Vec6<double> p_spatial_now,Vec6<double> p_spatial_dot_now,\
						Vec6<double> p_spatial_des,Vec6<double> p_spatial_dot_des);

	private:
		Vec6<double> spatial_now;
		Vec6<double> spatial_dot_now;

		Vec6<double> spatial_des;
		Vec6<double> spatial_dot_des;

	};


} 





#endif






