/**
 * @file legPlan.hpp
 * @brief Actiger腿部规划结果类LegPlan头文件
 * @author Bin & Tech13
 * @version 1.1
 * @date 2021-08-20
 * 
 * @copyright  Copyright  (c)  2021  Action
 * 
 */

#ifndef LEGPLAN_HPP
#define LEGPLAN_HPP

#include <vector>
#include "queue.hpp"
#include "robotModel.hpp"
#include "cppTypes.h"

namespace quadruped 
{
	/**
	 * @brief Actiger腿部规划结果类
	 */
	class LegPlan
	{
	private:

	protected:

	public:
		explicit LegPlan(void);
		virtual ~LegPlan(void);

		/** @brief 所有环形队列成员各自的容量 */
		uint32_t horizon;
		/** @brief 期望的地面反作用力[GRF] */
		Vec3<double> fr_des;
		/** @brief 装载期望的机体坐标系下的足端位置的环形队列 */
		Queue* pFoot_des;
		/** @brief 装载期望的机体坐标系下的足端速度的环形队列 */
		Queue* vFoot_des;
		/** @brief 装载期望的机体坐标系下的足端加速度的环形队列 */
		Queue* aFoot_des;
	};
}

#endif // !LEGPLAN_HPP