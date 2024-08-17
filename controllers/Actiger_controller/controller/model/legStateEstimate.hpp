/**
 * @file legStateEstimate.hpp
 * @brief Actiger单腿状态信息类LegStateEstimate头文件
 * @author Bin & Tech13
 * @version 1.1
 * @date 2021-08-20
 * 
 * @copyright  Copyright  (c)  2021  Action
 * 
 */

#ifndef LEGSTATEESTIMATE_HPP
#define LEGSTATEESTIMATE_HPP

#include <vector>
#include "parameters.hpp"
#include "cppTypes.h"
#include "cppStruct.h"

#ifdef ROBOT
    #include "embedded.hpp"
#else
	#include "webotsApi.hpp"
#endif

namespace quadruped
{
	/**
	 * @brief Actiger单腿状态信息类LegStateEstimate
	 */
	class LegStateEstimate
	{
	private:

	protected:

	public:
		explicit LegStateEstimate(int p_legNum);
		virtual ~LegStateEstimate(void);

		void Resize(void);
		void GetJointMsg(legMsg_t p_legMsg);
		void GetLegMsg(legMsg_t p_legMsg);
		void CaluEndfootMsg(void);
		void GetJacobian(void);
		void GetJdot(void);

		/** @brief 腿部索引 */
		int legNum;
		/** @brief 腿部第一段连杆 */
		double L0;
		/** @brief 足端触地状态 ： 1 - 触地  |  0 - 离地  */
		bool contact_state;
		/** @brief WBC预留接口 */
		Vec3<double> jdqd;
		/** @brief 雅可比矩阵 */
		Mat3<double> jacobian;
		/** @brief 足端相对于髋关节根部的位置 */
		Vec3<double> endfoot_hip;
		/** @brief 足端相对于髋关节根部的速度 */
		Vec3<double> endfoot_vel_hip;
		/** @brief 足端在惯性坐标系下的位置 */
		Vec3<double> endfoot_world;
		/** @brief 足端在惯性坐标系下的速度 */
		Vec3<double> endfoot_vel_world;
		/** @brief 所有关节角度 */
		Vec3<double> joint_angle;
		/** @brief 所有关节角速度 */
		Vec3<double> joint_anglularVel;
		/** @brief 髋关节在机体坐标系下的位置 */
		Vec3<double> hip_pos_body;	
	};
}

#endif // !LEGSTATEESTIMATE_HPP