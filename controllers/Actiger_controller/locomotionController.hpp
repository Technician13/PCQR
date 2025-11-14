#ifndef LOCOMOTION_CONTROLLER_HPP
#define LOCOMOTION_CONTROLLER_HPP

#include "parameters.hpp"
#include "legControl.hpp"
#include "mpc.hpp"
#include "cppTypes.h"

namespace quadruped
{
	class  LocomotionCtrl
	{
	private:
		
	protected:

	public:
		explicit LocomotionCtrl(void);
		virtual ~LocomotionCtrl(void);

		void RobotControl(void);

		Mpc* mpc;
		RobotModel* robotModel;
		Parameters* para;
		MotionPlan* motionPlan;
		LegControl* legControl;
		Vec12<double> mpc_pre_force = Vec12<double>::Zero();
		Vec12<double> mpc_pre_pre_force = Vec12<double>::Zero();
	};
}

#endif // !LOCOMOTION_CONTROLLER_HPP