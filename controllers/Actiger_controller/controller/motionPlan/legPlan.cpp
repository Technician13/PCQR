/**
 * @file legPlan.cpp
 * @brief Actiger腿部规划结果类LegPlan
 * @author Bin & Tech13
 * @version 1.1
 * @date 2021-08-20
 * 
 * @copyright  Copyright  (c)  2021  Action
 * 
 */

#include "legPlan.hpp"

using namespace std;
using namespace quadruped;

/**
 * @brief LegPlan的构造函数，完成所有对象成员的内存分配及数据成员的初值装载
 */
LegPlan::LegPlan()
{
	horizon = 251;
	fr_des.setZero();

	pFoot_des = new Queue(horizon);
	vFoot_des = new Queue(horizon);
	aFoot_des = new Queue(horizon);
}

/**
 * @brief LegPlan类的析构函数，释放申请的内存
 */
LegPlan::~LegPlan()
{
	delete pFoot_des;
	delete vFoot_des;
	delete aFoot_des;
}