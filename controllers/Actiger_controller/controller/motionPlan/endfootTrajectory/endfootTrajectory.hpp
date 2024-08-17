/**
 * @file endfootTrajectory.hpp
 * @brief Actiger摆动腿足端轨迹规划器EndfootTrajectory头文件
 * @author Bin & Tech13
 * @version 1.1
 * @date 2021-11-01
 * 
 * @copyright  Copyright  (c)  2021  Action
 * 
 */

#ifndef ENDFOOT_TRA_HPP
#define ENDFOOT_TRA_HPP

#include "terrain.hpp"
#include "commonFunction.hpp"
#include "parameters.hpp"
#include "miniSnap.hpp"

namespace quadruped
{
	/**
	 * @brief Actiger摆动腿足端轨迹规划器
	 */
	class EndfootTrajectory
	{
	private:

	protected:

	public:
		explicit EndfootTrajectory(Parameters* p_para,Terrain* p_terrain_now);
		virtual ~EndfootTrajectory(void);
		void GetEndfootTrajectory(int p_grid,
								  Vec3<double> p_front_startPoint,
								  Vec3<double> p_rear_startPoint,
								  Vec3<double> p_front_endPoint,
								  Vec3<double> p_rear_endPoint,
								  DMat<double> p_spatial_list);

		/** @brief 一次摆动过程中MPC周期数目 */
		int grid;
		/** @brief 当前地形 */
		Terrain* terrain_now;
		/** @brief Mininap类对象 */
		MiniSnap* minisnap;
		/** @brief 前摆动腿的起始点 */
		Vec3<double> front_startPoint;
		/** @brief 前摆动腿的终止点 */
		Vec3<double> front_endPoint;
		/** @brief 后摆动腿的起始点 */
		Vec3<double> rear_startPoint;
		/** @brief 后摆动腿的终止点 */
		Vec3<double> rear_endPoint;
		/** @brief 装载前摆动腿位置规划结果的列表 */
		DMat<double> endfoot_pos_front_list;
		/** @brief 装载前摆动腿速度规划结果的列表 */
		DMat<double> endfoot_vel_front_list;
		/** @brief 装载后摆动腿位置规划结果的列表 */
		DMat<double> endfoot_pos_rear_list;
		/** @brief 装载后摆动腿速度规划结果的列表 */
		DMat<double> endfoot_vel_rear_list;
	};
}

#endif