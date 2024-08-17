/**
 * @file locomotionController.hpp
 * @brief Actiger运动控制类LocomotionCtrl头文件
 * @author Bin & Tech13
 * @version 1.1
 * @date 2021-08-18
 * 
 * @copyright  Copyright  (c)  2021  Action
 * 
 */

#ifndef LOCOMOTION_CONTROLLER_HPP
#define LOCOMOTION_CONTROLLER_HPP

#include "parameters.hpp"
#include "legControl.hpp"
#include "mpc.hpp"
#include "cppTypes.h"

namespace quadruped
{
	/**
	 * @brief Actiger运动控制类
	 */
	class  LocomotionCtrl
	{
	private:
		
	protected:

	public:
		explicit LocomotionCtrl(void);
		virtual ~LocomotionCtrl(void);

		void RobotControl(void);

		/** @brief Mpc类的实例化对象，Actiger模型预测控制器 */
		Mpc* mpc;
		/** @brief RobotModel类的实例化对象，存储机器人模型信息 */
		RobotModel* robotModel;
		/** @brief Parameters类的实例化对象，存储一些被频繁使用的参数 */
		Parameters* para;
		/** @brief MotionPlan类的实例化对象，Actiger规划器 */
		MotionPlan* motionPlan;
		/** @brief LegControl类的实例化对象，Actiger腿部控制器 */
		LegControl* legControl;
		Vec12<double> mpc_pre_force = Vec12<double>::Zero();
		Vec12<double> mpc_pre_pre_force = Vec12<double>::Zero();
	};
}

#endif // !LOCOMOTION_CONTROLLER_HPP