#ifndef ENDFOOT_TRA_HPP
#define ENDFOOT_TRA_HPP

#include "terrain.hpp"
#include "commonFunction.hpp"
#include "parameters.hpp"
#include "miniSnap.hpp"

namespace quadruped
{
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

		int grid;
		Terrain* terrain_now;
		MiniSnap* minisnap;
		Vec3<double> front_startPoint;
		Vec3<double> front_endPoint;
		Vec3<double> rear_startPoint;
		Vec3<double> rear_endPoint;
		DMat<double> endfoot_pos_front_list;
		DMat<double> endfoot_vel_front_list;
		DMat<double> endfoot_pos_rear_list;
		DMat<double> endfoot_vel_rear_list;
	};
}

#endif