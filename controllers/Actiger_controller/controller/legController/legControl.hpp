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
	class LegControl
	{
	private:

	protected:

	public:
		explicit LegControl(RobotModel* p_robotModel, MotionPlan* p_motionPlan, Parameters* p_para);
		virtual ~LegControl();

		void LegJointControl(void);
		void SetParameter(void);

		SingleLeg* lf_leg;
		SingleLeg* lr_leg;
		SingleLeg* rf_leg;
		SingleLeg* rr_leg;
		RobotModel* robotModel;
		MotionPlan* motionPlan;	
	};
}

#endif // !LEGCONTROL_HPP