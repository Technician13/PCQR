/**
 * @file legControl.hpp
 * @brief Actiger腿部控制器头文件
 * @author Bin & Tech13
 * @version 1.1
 * @date 2021-08-20
 * 
 * @copyright  Copyright  (c)  2021  Action
 * 
 */

#ifndef LEGCONTROL_HPP
#define LEGCONTROL_HPP

#include <vector>
#include "singleLegControl.hpp"
#include "motionPlan.hpp"
#include "parameters.hpp"

#ifdef ROBOT
    #include "embedded.hpp" 
#else
	#include "webotsApi.hpp"
#endif

namespace quadruped 
{
	/**
	 * @brief Actiger腿部控制器
	 */
	class LegControl
	{
	private:

	protected:

	public:
		explicit LegControl(RobotModel* p_robotModel, MotionPlan* p_motionPlan, Parameters* p_para);
		virtual ~LegControl();

		void LegJointControl(void);
		void SetParameter(void);

		/** @brief SingleLeg类的实例化对象，Actiger左前腿单腿控制器 */
		SingleLeg* lf_leg;
		/** @brief SingleLeg类的实例化对象，Actiger左后腿单腿控制器 */
		SingleLeg* lr_leg;
		/** @brief SingleLeg类的实例化对象，Actiger右前腿单腿控制器 */
		SingleLeg* rf_leg;
		/** @brief SingleLeg类的实例化对象，Actiger右后腿单腿控制器 */
		SingleLeg* rr_leg;
		/** @brief RobotModel类的实例化对象，存储机器人模型信息 */
		RobotModel* robotModel;
		/** @brief MotionPlan类的实例化对象，Actiger规划器 */
		MotionPlan* motionPlan;	
	};
}

#endif // !LEGCONTROL_HPP