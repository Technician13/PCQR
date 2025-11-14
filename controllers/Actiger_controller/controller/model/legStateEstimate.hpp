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

		int legNum;
		double L0;
		bool contact_state;
		Vec3<double> jdqd;
		Mat3<double> jacobian;
		Vec3<double> endfoot_hip;
		Vec3<double> endfoot_vel_hip;
		Vec3<double> endfoot_world;
		Vec3<double> endfoot_vel_world;
		Vec3<double> joint_angle;
		Vec3<double> joint_anglularVel;
		Vec3<double> hip_pos_body;	
	};
}

#endif // !LEGSTATEESTIMATE_HPP