/**
 * @file bodyStateEstimate.hpp
 * @brief Actiger机体状态信息类BodyStateEstimate头文件
 * @author Bin & Tech13
 * @version 1.1
 * @date 2021-08-19
 * 
 * @copyright  Copyright  (c)  2021  Action
 * 
 */

#ifndef BODYSTATEESTIMATE_HPP
#define BODYSTATEESTIMATE_HPP

#include <vector>
#include "cppTypes.h"
#include "cppStruct.h"
#include "VelocityEstimate.hpp"
#include "legStateEstimate.hpp"

#ifdef ROBOT
    #include "embedded.hpp"
#else
	#include "webotsApi.hpp"
#endif

namespace quadruped 
{
	/**
	 * @brief Actiger机体状态信息类
	 */
	class BodyStateEstimate
	{
	private:
		void GetRobotBodyPos(bool p_LF_CONTACT,
                             bool p_LR_CONTACT,
                             bool p_RF_CONTACT,
                             bool p_RR_CONTACT,
                             Vec3<double> p_lf_endfoot_vel,
                             Vec3<double> p_lr_endfoot_vel,
                             Vec3<double> p_rf_endfoot_vel,
                             Vec3<double> p_rr_endfoot_vel,
                             Vec3<double> p_lf_endfoot,
                             Vec3<double> p_lr_endfoot,
                             Vec3<double> p_rf_endfoot,
                             Vec3<double> p_rr_endfoot,
                             DVec<double> p_f_k,
                             DVec<double> p_f_k_1,
                             Vec3<double> p_actul_body_acc);
		void GetRobotBodyAngle(void);
	
	protected:

	public:
		explicit BodyStateEstimate(void);
		virtual ~BodyStateEstimate(void);

		void GetRobotMsg(LegStateEstimate* p_lf_legState,
						 LegStateEstimate* p_lr_legState,
						 LegStateEstimate* p_rf_legState,
						 LegStateEstimate* p_rr_legState,
						 Vec12<double> p_f_k,
						 Vec12<double> p_f_k_1,
					     Vec3<double> p_actul_body_acc);

		/** @brief Actiger在全局坐标系下的实际位置 */
		Vec3<double> actul_body_pos;
		/** @brief Actiger在惯性坐标系下的实际速度 */
		Vec3<double> actul_body_vel;
		/** @brief Actiger在惯性坐标系下的实际欧拉角 */
		Vec3<double> actul_body_angle;
		/** @brief Actiger在惯性坐标系下的实际欧拉角速度 */
		Vec3<double> actul_body_angularVel;
		/** @brief Actiger从机体到世界的旋转矩阵 */
		Mat3<double> rotation;

		//for position estimate
		double z_offset;
        double position_x_,position_y_,position_z_;
        Vec3<double> bodyPosWorld;
        int cnt;
		
		VelocityEstimate* velocityEstimate;
	};
}

#endif // !BODYSTATEESTIMATE_HPP